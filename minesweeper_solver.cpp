// minesweeper_solver.cpp
// ------------------------------------------------------------
// Screenshot -> OCR (หาตาราง + อ่านเลข/ธง) -> คำนวณความน่าจะเป็นของทุกช่องที่ยังไม่เปิด
//
// Build : g++ -O2 -std=c++17 minesweeper_solver.cpp -o ms_solver
//         (ต้องมี stb_image.h และ stb_image_write.h อยู่โฟลเดอร์เดียวกัน
//          https://github.com/nothings/stb)
// Usage : ./ms_solver pic.png [--edges tr] [--density 0.206] [--cell 30]
//                              [--no-flags] [--out annotated.png] [--debug]
//
//   --edges  ด้านที่ "ตัน" = เป็นขอบกระดานจริง (ไม่มีช่องต่อออกไปอีก)
//            t=บน r=ขวา b=ล่าง l=ซ้าย  เช่น  tr / tl / trbl / none
//            ด้านที่ไม่ได้ระบุ = ภาพถูกตัด มีช่องต่อออกไปนอกภาพ
//            (โปรแกรมจะเติมช่องปริศนา 1 ชั้นให้ตรงนั้นเอง)
//   --density  ความหนาแน่นของทุ่นระเบิดโดยรวม (Beginner .123 / Inter. .156 / Expert .206)
//   --cell     บังคับขนาดช่อง (px) ถ้าตรวจอัตโนมัติผิด
//   --no-flags ไม่เชื่อธง (ถือว่าธงเป็นช่องปิดธรรมดา)
// ------------------------------------------------------------
#include <bits/stdc++.h>
using namespace std;

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

enum { COVERED = -1, FLAG = -2 };

// ======================= Image =======================
struct Img {
    int w = 0, h = 0;
    vector<unsigned char> d;
    array<int, 3> at(int x, int y) const {
        x = clamp(x, 0, w - 1);
        y = clamp(y, 0, h - 1);
        const unsigned char* p = &d[((size_t)y * w + x) * 3];
        return {p[0], p[1], p[2]};
    }
    void set(int x, int y, int r, int g, int b) {
        if (x < 0 || y < 0 || x >= w || y >= h) return;
        unsigned char* p = &d[((size_t)y * w + x) * 3];
        p[0] = r; p[1] = g; p[2] = b;
    }
};

static bool loadImg(const string& path, Img& im) {
    int n;
    unsigned char* p = stbi_load(path.c_str(), &im.w, &im.h, &n, 3);
    if (!p) return false;
    im.d.assign(p, p + (size_t)im.w * im.h * 3);
    stbi_image_free(p);
    return true;
}

// ======================= Grid detection =======================
struct Axis { double pitch = 0, phase = 0, score = -1e18; };

static double gapScore(const vector<double>& prof, double mean, double p, double f) {
    int n = prof.size(), cnt = 0;
    double s = 0;
    for (double g = f; g < n - 1; g += p) {
        int gi = (int)lround(g);
        if (gi < 1 || gi > n - 2) continue;
        s += (prof[gi - 1] + prof[gi] + prof[gi + 1]) / 3.0;
        cnt++;
    }
    if (cnt < 3) return -1e18;
    return mean - s / cnt;  // เส้นแบ่งช่องมืดกว่าค่าเฉลี่ย -> score สูง
}

static Axis bestAxis(const vector<double>& prof, double mean, double pLo, double pHi, double fixedP = 0) {
    Axis best;
    for (double p = fixedP > 0 ? fixedP : pLo; p <= (fixedP > 0 ? fixedP : pHi) + 1e-9; p += 0.1)
        for (double f = 0; f < p; f += 0.25) {
            double s = gapScore(prof, mean, p, f);
            if (s > best.score) best = {p, f, s};
        }
    return best;
}

static Axis detectAxis(const vector<double>& prof, double forcedPitch) {
    double mean = accumulate(prof.begin(), prof.end(), 0.0) / prof.size();
    int n = prof.size();
    if (forcedPitch > 0) return bestAxis(prof, mean, 0, 0, forcedPitch);
    Axis top = bestAxis(prof, mean, 12, n / 3.5);
    Axis res = top;
    // ถ้าได้ pitch เป็นพหุคูณของจริง ลองหารลง
    for (int d = 5; d >= 2; d--) {
        double c = top.pitch / d;
        if (c < 10) continue;
        Axis a = bestAxis(prof, mean, c - 1.2, c + 1.2);
        if (a.score >= 0.85 * top.score) { res = a; break; }
    }
    return res;
}

static vector<pair<double, double>> cellsAlong(const Axis& a, int n) {
    vector<pair<double, double>> v;
    for (int k = -1;; k++) {
        double s = a.phase + k * a.pitch, e = s + a.pitch;
        if (s >= n) break;
        double vis = (min<double>(e, n) - max(0.0, s)) / a.pitch;
        if (vis >= 0.55) v.push_back({s, e});
    }
    return v;
}

// ======================= OCR =======================
struct Pal { int n; int r, g, b; };
static const Pal PALETTE[] = {
    {1, 124, 199, 255}, {2, 102, 194, 102}, {3, 255, 119, 136}, {4, 238, 136, 255},
    {5, 255, 180, 90},  {6, 90, 220, 210},  {7, 240, 240, 130}, {8, 200, 200, 200}};  // 5-8 เป็นค่าเดา ปรับได้

static int readCell(const Img& im, double x0, double x1, double y0, double y1, double p, bool dbg) {
    x0 = max(0.0, x0); y0 = max(0.0, y0);
    x1 = min<double>(im.w, x1); y1 = min<double>(im.h, y1);
    double cx = (x0 + x1) / 2, cy = (y0 + y1) / 2, half = 0.30 * p;

    // สีพื้นหลัง: median ความสว่างที่มุมของช่อง (เลี่ยงตัวเลข)
    vector<double> L;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int a = -1; a <= 1; a++)
                for (int b = -1; b <= 1; b++) {
                    double x = clamp(cx + sx * half + a, x0 + 2, x1 - 2);
                    double y = clamp(cy + sy * half + b, y0 + 2, y1 - 2);
                    auto c = im.at((int)x, (int)y);
                    L.push_back((c[0] + c[1] + c[2]) / 3.0);
                }
    nth_element(L.begin(), L.begin() + L.size() / 2, L.end());
    double bg = L[L.size() / 2];

    int white = 0;
    vector<array<int, 4>> sat;  // r,g,b,saturation
    for (int y = (int)(cy - half); y <= (int)(cy + half); y++)
        for (int x = (int)(cx - half); x <= (int)(cx + half); x++) {
            auto c = im.at(x, y);
            int mn = min({c[0], c[1], c[2]}), mx = max({c[0], c[1], c[2]});
            if (mn > 200) white++;
            else if (mx - mn > 60) sat.push_back({c[0], c[1], c[2], mx - mn});
        }
    int thr = max(6, (int)(0.015 * p * p));
    bool revealed = bg < 70;  // ช่องที่เปิดแล้วเข้มกว่าช่องปิด (ธีมมืดแบบในภาพ)

    if (dbg) fprintf(stderr, "  bg=%.0f white=%d sat=%zu\n", bg, white, sat.size());
    if (!revealed) return white >= thr ? FLAG : COVERED;
    if ((int)sat.size() < thr) return 0;

    int maxS = 0;
    for (auto& s : sat) maxS = max(maxS, s[3]);
    double R = 0, G = 0, B = 0; int k = 0;
    for (auto& s : sat) if (s[3] >= 0.9 * maxS) { R += s[0]; G += s[1]; B += s[2]; k++; }
    R /= k; G /= k; B /= k;
    int best = 1; double bd = 1e18;
    for (auto& q : PALETTE) {
        double d = (R - q.r) * (R - q.r) + (G - q.g) * (G - q.g) + (B - q.b) * (B - q.b);
        if (d < bd) { bd = d; best = q.n; }
    }
    return best;
}

// ======================= Board / Solver =======================
struct Board {
    int R = 0, C = 0;
    vector<int> st;      // COVERED / FLAG / 0..8
    vector<char> virt;   // ช่องปริศนานอกภาพ
    int id(int r, int c) const { return r * C + c; }
    bool in(int r, int c) const { return r >= 0 && r < R && c >= 0 && c < C; }
};

struct CompSolver {
    int nv = 0;
    vector<vector<int>> vc;  // var -> constraints
    vector<int> need, mines, unas;
    vector<char> asg;
    vector<double> counts;
    vector<vector<double>> mineAt;
    long long nodes = 0, limit = 300000000LL;
    bool trunc = false;

    void rec(int v, int m) {
        if (trunc) return;
        if (++nodes > limit) { trunc = true; return; }
        if (v == nv) {
            counts[m] += 1;
            for (int i = 0; i < nv; i++) if (asg[i]) mineAt[i][m] += 1;
            return;
        }
        for (int x = 0; x < 2; x++) {
            bool ok = true;
            int k = 0;
            for (; k < (int)vc[v].size(); k++) {
                int ci = vc[v][k];
                mines[ci] += x; unas[ci]--;
                if (mines[ci] > need[ci] || mines[ci] + unas[ci] < need[ci]) { ok = false; k++; break; }
            }
            if (ok) { asg[v] = x; rec(v + 1, m + x); asg[v] = 0; }
            for (int j = 0; j < k; j++) { int ci = vc[v][j]; mines[ci] -= x; unas[ci]++; }
        }
    }
};

// คืน prob[idx]: 0..1 สำหรับช่องปิด, -1 สำหรับช่องที่ไม่ใช่ช่องปิด
static vector<double> solveProb(const Board& b, bool trustFlags, double density,
                                vector<string>& warn, bool& truncated, int& nFrontier, int& nComps) {
    int N = b.R * b.C;
    vector<double> prob(N, -1.0);
    vector<int> varOf(N, -1), cellOf;
    struct Cons { vector<int> vars; int need; };
    vector<Cons> cons;

    for (int r = 0; r < b.R; r++)
        for (int c = 0; c < b.C; c++) {
            int s = b.st[b.id(r, c)];
            if (s < 0) continue;
            Cons cs; cs.need = s;
            int flags = 0;
            for (int dr = -1; dr <= 1; dr++)
                for (int dc = -1; dc <= 1; dc++) {
                    if (!dr && !dc) continue;
                    int rr = r + dr, cc = c + dc;
                    if (!b.in(rr, cc)) continue;
                    int t = b.st[b.id(rr, cc)];
                    if (t == COVERED || (t == FLAG && !trustFlags)) {
                        int i = b.id(rr, cc);
                        if (varOf[i] < 0) { varOf[i] = cellOf.size(); cellOf.push_back(i); }
                        cs.vars.push_back(varOf[i]);
                    } else if (t == FLAG) flags++;
                }
            cs.need -= flags;
            if (cs.vars.empty()) {
                if (cs.need != 0) warn.push_back("ช่อง (" + to_string(r + 1) + "," + to_string(c + 1) + ") เลขไม่ตรงกับธงรอบข้าง (OCR ผิด/ปักธงผิด?)");
                continue;
            }
            if (cs.need < 0 || cs.need > (int)cs.vars.size()) {
                warn.push_back("ช่อง (" + to_string(r + 1) + "," + to_string(c + 1) + ") ข้อมูลขัดแย้ง ข้ามเงื่อนไขนี้");
                continue;
            }
            cons.push_back(cs);
        }

    int nv = cellOf.size();
    nFrontier = nv;
    // union-find
    vector<int> par(nv);
    iota(par.begin(), par.end(), 0);
    function<int(int)> fnd = [&](int x) { return par[x] == x ? x : par[x] = fnd(par[x]); };
    for (auto& cs : cons)
        for (size_t i = 1; i < cs.vars.size(); i++) par[fnd(cs.vars[i])] = fnd(cs.vars[0]);

    vector<vector<int>> varCons(nv);
    for (int ci = 0; ci < (int)cons.size(); ci++)
        for (int v : cons[ci].vars) varCons[v].push_back(ci);

    // ช่องปิดทั้งหมด: ค่าเริ่มต้น = density (ช่องนอกขอบเขต) / 1 สำหรับธง
    for (int i = 0; i < N; i++) {
        if (b.st[i] == COVERED) prob[i] = density;
        else if (b.st[i] == FLAG) prob[i] = trustFlags ? 1.0 : density;
    }

    double w = density / (1.0 - density);
    map<int, vector<int>> comps;
    for (int v = 0; v < nv; v++) comps[fnd(v)].push_back(v);
    nComps = comps.size();

    for (auto& kv : comps) {
        // BFS เรียงตัวแปรให้เงื่อนไขปิดเร็ว
        const vector<int>& members = kv.second;
        vector<int> order, loc(nv, -1);
        queue<int> q;
        q.push(members[0]); loc[members[0]] = 0; order.push_back(members[0]);
        while (!q.empty()) {
            int v = q.front(); q.pop();
            for (int ci : varCons[v])
                for (int u : cons[ci].vars)
                    if (loc[u] < 0) { loc[u] = order.size(); order.push_back(u); q.push(u); }
        }
        CompSolver cs;
        cs.nv = order.size();
        cs.vc.assign(cs.nv, {});
        map<int, int> consLocal;
        for (int v : order)
            for (int ci : varCons[v]) {
                if (!consLocal.count(ci)) {
                    int li = consLocal.size();
                    consLocal[ci] = li;
                    cs.need.push_back(cons[ci].need);
                    cs.unas.push_back(cons[ci].vars.size());
                }
                cs.vc[loc[v]].push_back(consLocal[ci]);
            }
        cs.mines.assign(cs.need.size(), 0);
        cs.asg.assign(cs.nv, 0);
        cs.counts.assign(cs.nv + 1, 0);
        cs.mineAt.assign(cs.nv, vector<double>(cs.nv + 1, 0));
        cs.rec(0, 0);
        if (cs.trunc) truncated = true;

        int mmin = -1;
        for (int m = 0; m <= cs.nv; m++) if (cs.counts[m] > 0) { mmin = m; break; }
        if (mmin < 0) { warn.push_back("กลุ่มช่องหนึ่งไม่มีคำตอบที่เป็นไปได้ (อ่านเลขผิด?)"); continue; }
        double den = 0;
        for (int m = mmin; m <= cs.nv; m++) den += cs.counts[m] * pow(w, m - mmin);
        for (int i = 0; i < cs.nv; i++) {
            double num = 0;
            for (int m = mmin; m <= cs.nv; m++) num += cs.mineAt[i][m] * pow(w, m - mmin);
            prob[cellOf[order[i]]] = num / den;
        }
    }
    return prob;
}

// ======================= tiny font for overlay =======================
static const char* GLYPH[] = {  // 3x5, ตัวเลข 0-9 และ X (index 10)
    "111101101101111", "010110010010111", "111001111100111", "111001111001111",
    "101101111001001", "111100111001111", "111100111101111", "111001001001001",
    "111101111101111", "111101111001111", "101101010101101"};

static void drawGlyph(Img& im, int g, int x, int y, int s, int r, int gg, int b) {
    for (int j = 0; j < 5; j++)
        for (int i = 0; i < 3; i++)
            if (GLYPH[g][j * 3 + i] == '1')
                for (int a = 0; a < s; a++)
                    for (int c = 0; c < s; c++) im.set(x + i * s + a, y + j * s + c, r, gg, b);
}

static void drawLabel(Img& im, const string& t, int cx, int cy, int s) {
    int wTxt = (int)t.size() * 4 * s - s, hTxt = 5 * s;
    int x0 = cx - wTxt / 2, y0 = cy - hTxt / 2;
    for (int y = y0 - 2; y < y0 + hTxt + 2; y++)
        for (int x = x0 - 2; x < x0 + wTxt + 2; x++) im.set(x, y, 0, 0, 0);
    for (size_t i = 0; i < t.size(); i++) {
        int g = t[i] == 'X' ? 10 : t[i] - '0';
        drawGlyph(im, g, x0 + (int)i * 4 * s, y0, s, 255, 255, 255);
    }
}

// ======================= main =======================
int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s image.png [--edges tr|none|...] [--density 0.206] [--cell px] [--no-flags] [--out out.png] [--debug]\n", argv[0]);
        return 1;
    }
    string path = argv[1], edgesArg = "none", outPath = "annotated.png";
    double density = 0.206, forcedPitch = 0;
    bool trustFlags = true, debug = false;
    for (int i = 2; i < argc; i++) {
        string a = argv[i];
        if (a == "--edges" && i + 1 < argc) edgesArg = argv[++i];
        else if (a == "--density" && i + 1 < argc) density = atof(argv[++i]);
        else if (a == "--cell" && i + 1 < argc) forcedPitch = atof(argv[++i]);
        else if (a == "--out" && i + 1 < argc) outPath = argv[++i];
        else if (a == "--no-flags") trustFlags = false;
        else if (a == "--debug") debug = true;
    }
    density = clamp(density, 0.001, 0.95);

    // parse edges
    string e = edgesArg;
    for (auto& ch : e) ch = tolower(ch);
    auto repl = [&](const string& from, const string& to) {
        size_t p;
        while ((p = e.find(from)) != string::npos) e.replace(p, from.size(), to);
    };
    repl("none", ""); repl("bottom", "b"); repl("right", "r"); repl("left", "l"); repl("top", "t");
    bool eT = e.find('t') != string::npos, eR = e.find('r') != string::npos;
    bool eB = e.find('b') != string::npos, eL = e.find('l') != string::npos;

    Img im;
    if (!loadImg(path, im)) { fprintf(stderr, "โหลดภาพไม่ได้: %s\n", path.c_str()); return 1; }

    // ---- 1) หาตาราง ----
    vector<double> colP(im.w, 0), rowP(im.h, 0);
    for (int y = 0; y < im.h; y++)
        for (int x = 0; x < im.w; x++) {
            auto c = im.at(x, y);
            double l = (c[0] + c[1] + c[2]) / 3.0;
            colP[x] += l / im.h;
            rowP[y] += l / im.w;
        }
    Axis ax = detectAxis(colP, forcedPitch), ay = detectAxis(rowP, forcedPitch);
    auto xs = cellsAlong(ax, im.w), ys = cellsAlong(ay, im.h);
    int H = ys.size(), W = xs.size();
    printf("ภาพ %dx%d  | ช่อง ~%.1fx%.1f px | ตาราง %d แถว x %d คอลัมน์\n", im.w, im.h, ay.pitch, ax.pitch, H, W);
    if (H == 0 || W == 0) { fprintf(stderr, "หาตารางไม่เจอ ลองใส่ --cell\n"); return 1; }

    // ---- 2) OCR ----
    vector<vector<int>> grid(H, vector<int>(W));
    double pavg = (ax.pitch + ay.pitch) / 2;
    for (int r = 0; r < H; r++)
        for (int c = 0; c < W; c++) {
            if (debug) fprintf(stderr, "cell (%d,%d)\n", r + 1, c + 1);
            grid[r][c] = readCell(im, xs[c].first, xs[c].second, ys[r].first, ys[r].second, pavg, debug);
        }

    printf("\n=== OCR (# = ช่องปิด, F = ธง, . = 0) ===\n    ");
    for (int c = 0; c < W; c++) printf("%2d ", c + 1);
    printf("\n");
    for (int r = 0; r < H; r++) {
        printf("%2d  ", r + 1);
        for (int c = 0; c < W; c++) {
            int v = grid[r][c];
            char ch = v == COVERED ? '#' : v == FLAG ? 'F' : v == 0 ? '.' : '0' + v;
            printf(" %c ", ch);
        }
        printf("\n");
    }

    // ---- 3) สร้างกระดาน + ช่องปริศนาด้านที่ภาพถูกตัด ----
    int pT = eT ? 0 : 1, pB = eB ? 0 : 1, pL = eL ? 0 : 1, pR = eR ? 0 : 1;
    Board bd;
    bd.R = H + pT + pB;
    bd.C = W + pL + pR;
    bd.st.assign(bd.R * bd.C, COVERED);
    bd.virt.assign(bd.R * bd.C, 1);
    for (int r = 0; r < H; r++)
        for (int c = 0; c < W; c++) {
            bd.st[bd.id(r + pT, c + pL)] = grid[r][c];
            bd.virt[bd.id(r + pT, c + pL)] = 0;
        }
    printf("\nขอบตัน: top=%d right=%d bottom=%d left=%d | density=%.3f | flags=%s\n", eT, eR, eB, eL, density,
           trustFlags ? "เชื่อ" : "ไม่เชื่อ");

    // ---- 4) คำนวณความน่าจะเป็น ----
    vector<string> warn;
    bool trunc = false;
    int nFront = 0, nComp = 0;
    vector<double> prob = solveProb(bd, trustFlags, density, warn, trunc, nFront, nComp);
    printf("ช่องขอบเขต (frontier) %d ช่อง, %d กลุ่มอิสระ\n", nFront, nComp);
    for (auto& s : warn) printf("[เตือน] %s\n", s.c_str());
    if (trunc) printf("[เตือน] กลุ่มใหญ่เกินไป ค้นหาไม่ครบ ผลเป็นค่าประมาณ\n");

    printf("\n=== โอกาสเป็นระเบิด (%%)  | [  x ] = ช่องที่ไม่ได้ติดเลขเลย ใช้ค่า density ===\n    ");
    for (int c = 0; c < W; c++) printf("%4d ", c + 1);
    printf("\n");
    vector<int> isFront(bd.R * bd.C, 0);
    {
        // ช่องที่อยู่ติดเลขอย่างน้อย 1 ตัว
        for (int r = 0; r < bd.R; r++)
            for (int c = 0; c < bd.C; c++)
                if (bd.st[bd.id(r, c)] >= 0)
                    for (int dr = -1; dr <= 1; dr++)
                        for (int dc = -1; dc <= 1; dc++)
                            if (bd.in(r + dr, c + dc)) isFront[bd.id(r + dr, c + dc)] = 1;
    }
    for (int r = 0; r < H; r++) {
        printf("%2d  ", r + 1);
        for (int c = 0; c < W; c++) {
            int i = bd.id(r + pT, c + pL), v = bd.st[i];
            if (v == FLAG) printf("  F  ");
            else if (v >= 0) printf("  .  ");
            else if (!isFront[i]) printf("[%2d] ", (int)lround(prob[i] * 100));
            else if (prob[i] < 1e-9) printf("  0  ");
            else if (prob[i] > 1 - 1e-9) printf(" MINE");
            else printf("%4.0f ", prob[i] * 100);
        }
        printf("\n");
    }

    // ---- 5) สรุปคำแนะนำ ----
    struct Cand { int r, c; double p; bool front; };
    vector<Cand> cands, safe, mine;
    for (int r = 0; r < H; r++)
        for (int c = 0; c < W; c++) {
            int i = bd.id(r + pT, c + pL);
            if (bd.st[i] != COVERED) continue;
            Cand cd{r, c, prob[i], (bool)isFront[i]};
            cands.push_back(cd);
            if (prob[i] < 1e-9) safe.push_back(cd);
            if (prob[i] > 1 - 1e-9) mine.push_back(cd);
        }
    printf("\n=== สรุป ===\n");
    if (!safe.empty()) {
        printf("ช่องปลอดภัย 100%% (กดได้เลย): ");
        for (auto& s : safe) printf("(แถว%d,หลัก%d) ", s.r + 1, s.c + 1);
        printf("\n");
    }
    if (!mine.empty()) {
        printf("ช่องที่เป็นระเบิดแน่นอน (ปักธง): ");
        for (auto& s : mine) printf("(แถว%d,หลัก%d) ", s.r + 1, s.c + 1);
        printf("\n");
    }
    if (safe.empty()) {
        sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) { return a.p < b.p; });
        printf("ไม่มีช่องปลอดภัยแน่นอน -> ต้องเดา เรียงจากเสี่ยงน้อยสุด:\n");
        for (size_t i = 0; i < min<size_t>(5, cands.size()); i++)
            printf("  แถว%d หลัก%d  เสี่ยง %.1f%%%s\n", cands[i].r + 1, cands[i].c + 1, cands[i].p * 100,
                   cands[i].front ? "" : "  (ช่องไกลเลข ใช้ density)");
    }

    // ---- 6) วาดภาพผลลัพธ์ ----
    Img out = im;
    int sc = max(1, (int)lround(pavg / 16.0));
    for (int r = 0; r < H; r++)
        for (int c = 0; c < W; c++) {
            int i = bd.id(r + pT, c + pL);
            if (bd.st[i] != COVERED) continue;
            double p = prob[i];
            int cr, cg, cb;
            if (p < 1e-9) { cr = 0; cg = 220; cb = 0; }
            else if (p > 1 - 1e-9) { cr = 230; cg = 0; cb = 0; }
            else {
                double t = min(1.0, p * 2);  // 0 -> เขียว, 0.5+ -> แดง
                cr = (int)(255 * t); cg = (int)(220 * (1 - t) + 120 * t * 0); cb = 0;
                if (!isFront[i]) { cr = 90; cg = 90; cb = 160; }
            }
            int x0 = (int)max(0.0, xs[c].first) + 3, x1 = (int)min<double>(im.w, xs[c].second) - 3;
            int y0 = (int)max(0.0, ys[r].first) + 3, y1 = (int)min<double>(im.h, ys[r].second) - 3;
            for (int y = y0; y < y1; y++)
                for (int x = x0; x < x1; x++) {
                    auto o = im.at(x, y);
                    out.set(x, y, (o[0] + cr) / 2, (o[1] + cg) / 2, (o[2] + cb) / 2);
                }
            string t = p > 1 - 1e-9 ? "X" : to_string(min(99, (int)lround(p * 100)));
            drawLabel(out, t, (x0 + x1) / 2, (y0 + y1) / 2, sc);
        }
    stbi_write_png(outPath.c_str(), out.w, out.h, 3, out.d.data(), out.w * 3);
    printf("\nบันทึกภาพผลลัพธ์: %s\n", outPath.c_str());
    return 0;
}
