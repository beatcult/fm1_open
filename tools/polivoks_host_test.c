/* SPDX-License-Identifier: GPL-3.0-only WITH GPL-3.0-linking-exception
 * Host-side checks for app/polivoks.c. Build and run (Linux/macOS):
 *   cc -O2 -DFM1_HOST_TEST -Iapp -Iapp/synth tools/polivoks_host_test.c -o polivoks_host_test
 *   ./polivoks_host_test [output_dir]
 * Writes 16-bit mono WAVs for listening/analysis and exits non-zero on failure. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../app/polivoks.c"

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { ++failures; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void wav(const char *dir, const char *name, const int32_t *s, size_t n)
{
    char path[512]; snprintf(path, sizeof path, "%s/%s.wav", dir, name);
    FILE *f = fopen(path, "wb"); if (!f) return;
    uint32_t bytes = (uint32_t)(n * 2), rate = 44100, br = rate * 2, riff = 36 + bytes;
    uint16_t fmt = 1, ch = 1, align = 2, bits = 16; uint32_t sixteen = 16;
    fwrite("RIFF", 1, 4, f); fwrite(&riff, 4, 1, f); fwrite("WAVEfmt ", 1, 8, f); fwrite(&sixteen, 4, 1, f);
    fwrite(&fmt, 2, 1, f); fwrite(&ch, 2, 1, f); fwrite(&rate, 4, 1, f); fwrite(&br, 4, 1, f);
    fwrite(&align, 2, 1, f); fwrite(&bits, 2, 1, f); fwrite("data", 1, 4, f); fwrite(&bytes, 4, 1, f);
    for (size_t i = 0; i < n; ++i) { int v = s[i] >> 8; if (v > 32767) v = 32767; if (v < -32768) v = -32768; int16_t w = (int16_t)v; fwrite(&w, 2, 1, f); }
    fclose(f);
}
static int32_t *run(polivoks_bank_t *b, unsigned blocks, double *peak, double *rms)
{
    int32_t *out = calloc((size_t)blocks * POLIVOKS_FRAMES, 4);
    double p = 0, e = 0;
    for (unsigned i = 0; i < blocks; ++i) {
        polivoks_bank_render(b, out + (size_t)i * POLIVOKS_FRAMES);
        for (unsigned j = 0; j < POLIVOKS_FRAMES; ++j) { double x = out[(size_t)i * POLIVOKS_FRAMES + j] / Q23; if (fabs(x) > p) p = fabs(x); e += x * x; }
    }
    if (peak) *peak = p;
    if (rms) *rms = sqrt(e / ((double)blocks * POLIVOKS_FRAMES));
    return out;
}
static polivoks_bank_t *fresh(void) { polivoks_bank_t *b = aligned_alloc(32, (polivoks_bank_size() + 31) & ~31u); polivoks_bank_init(b); return b; }

int main(int argc, char **argv)
{
    const char *dir = argc > 1 ? argv[1] : ".";
    double peak, rms;
    polivoks_bank_t *b = fresh();

    /* 1. duophonic chord, release to silence */
    polivoks_bank_note(b, 48, 100, 0); polivoks_bank_note(b, 55, 100, 0);
    int32_t *a = run(b, 344, &peak, &rms);
    printf("chord: peak %.3f rms %.3f active %u\n", peak, rms, polivoks_bank_active(b));
    CHECK(peak > 0.05 && peak < 0.95, "chord level out of range: %f", peak);
    CHECK(polivoks_bank_active(b) == 2, "two voices expected");
    wav(dir, "chord_default", a, 344 * POLIVOKS_FRAMES); free(a);
    polivoks_bank_note(b, 48, 0, 0); polivoks_bank_note(b, 55, 0, 0);
    a = run(b, 700, &peak, &rms); free(a);
    CHECK(polivoks_bank_active(b) == 0, "voices must go idle after release (active %u)", polivoks_bank_active(b));

    /* 2. note stack: the two newest held notes sound; releasing the newest brings the older one back */
    polivoks_bank_init(b);
    polivoks_bank_note(b, 48, 100, 0); polivoks_bank_note(b, 52, 100, 0); polivoks_bank_note(b, 55, 100, 0);
    int gated48 = 0, gated52 = 0, gated55 = 0;
    for (int i = 0; i < 2; ++i) { if (b->v[i].gate) { gated48 += b->v[i].note == 48; gated52 += b->v[i].note == 52; gated55 += b->v[i].note == 55; } }
    CHECK(!gated48 && gated52 && gated55, "after 48,52,55 expect 52+55 (got %d %d %d)", gated48, gated52, gated55);
    polivoks_bank_note(b, 55, 0, 0);
    gated48 = gated52 = 0;
    for (int i = 0; i < 2; ++i) { if (b->v[i].gate) { gated48 += b->v[i].note == 48; gated52 += b->v[i].note == 52; } }
    CHECK(gated48 && gated52, "after releasing 55 expect 48+52 (got %d %d)", gated48, gated52);
    polivoks_bank_pedal(b, 1); polivoks_bank_note(b, 52, 0, 0); polivoks_bank_note(b, 48, 0, 0);
    CHECK(polivoks_bank_active(b) == 2 && b->v[0].gate + b->v[1].gate == 2, "pedal must hold released notes");
    polivoks_bank_pedal(b, 0);
    CHECK(b->v[0].gate + b->v[1].gate == 0, "pedal up must release");

    /* 3. filter: lower cutoff must darken the sound (ratio of first-difference energy to signal energy) */
    double brightness[2];
    for (int pass = 0; pass < 2; ++pass) {
        polivoks_bank_init(b);
        polivoks_bank_set(b, POLIVOKS_CUTOFF, pass ? 20 : 120); polivoks_bank_set(b, POLIVOKS_ENVAMT, 0);
        polivoks_bank_set(b, POLIVOKS_SUSTAIN, 127); polivoks_bank_set(b, POLIVOKS_DETUNE, 64);
        polivoks_bank_note(b, 45, 127, 0);
        a = run(b, 100, 0, 0); free(a);
        a = run(b, 50, &peak, &rms);
        double d = 0, e = 0;
        for (unsigned i = 1; i < 50 * POLIVOKS_FRAMES; ++i) { double x = a[i] / Q23, dx = (a[i] - a[i - 1]) / Q23; d += dx * dx; e += x * x; }
        brightness[pass] = sqrt(d / e); free(a);
        printf("cutoff %s: brightness %.4f (peak %.3f)\n", pass ? "low" : "high", brightness[pass], peak);
    }
    CHECK(brightness[1] * 4 < brightness[0], "low cutoff should be much darker than high cutoff");

    /* 3b. looping envelope: with Loop on and no sustain the attack restarts while the key is held */
    for (int pass = 0; pass < 2; ++pass) {
        polivoks_bank_init(b);
        polivoks_bank_set(b, POLIVOKS_SUSTAIN, 0); polivoks_bank_set(b, POLIVOKS_DECAY, 40);
        polivoks_bank_set(b, POLIVOKS_ATTACK, 0); polivoks_bank_set(b, POLIVOKS_LOOP, pass ? POLIVOKS_LOOP_AMP : POLIVOKS_LOOP_OFF);
        polivoks_bank_note(b, 48, 110, 0);
        unsigned restarts = 0, last = 0;
        for (int blk = 0; blk < 700; ++blk) {
            int32_t tmp[POLIVOKS_FRAMES] = {0}; polivoks_bank_render(b, tmp);
            unsigned st = b->v[0].gate ? b->v[0].stage : b->v[1].stage;
            if (st == 1 && last == 2) ++restarts;
            last = st;
        }
        printf("loop %s: %u restarts in 2 s\n", pass ? "on" : "off", restarts);
        if (pass) CHECK(restarts >= 3, "looping envelope should restart repeatedly (%u)", restarts);
        else CHECK(restarts == 0, "no looping expected with Loop off (%u)", restarts);
    }

    /* 3c. LFO on the filter makes the brightness move over time; depth 0 does not */
    double swing[2];
    for (int pass = 0; pass < 2; ++pass) {
        polivoks_bank_init(b);
        polivoks_bank_set(b, POLIVOKS_ENVAMT, 0); polivoks_bank_set(b, POLIVOKS_SUSTAIN, 127); polivoks_bank_set(b, POLIVOKS_FSUSTAIN, 0);
        polivoks_bank_set(b, POLIVOKS_LFOTARGET, POLIVOKS_TARGET_FILTER); polivoks_bank_set(b, POLIVOKS_LFORATE, 60);
        polivoks_bank_set(b, POLIVOKS_LFODEPTH, pass ? 127 : 0); polivoks_bank_set(b, POLIVOKS_CUTOFF, 60);
        polivoks_bank_note(b, 45, 127, 0);
        double lo = 1e9, hi = 0;
        for (int blk = 0; blk < 800; ++blk) {
            int32_t tmp[POLIVOKS_FRAMES] = {0}; polivoks_bank_render(b, tmp);
            if (blk < 100) continue;
            double d = 0, e = 1e-12;
            for (unsigned i = 1; i < POLIVOKS_FRAMES; ++i) { double x = tmp[i] / Q23, dx = (tmp[i] - tmp[i - 1]) / Q23; d += dx * dx; e += x * x; }
            double br = sqrt(d / e); if (br < lo) lo = br; if (br > hi) hi = br;
        }
        swing[pass] = hi / lo;
        printf("lfo depth %s: brightness swing x%.2f\n", pass ? "full" : "zero", swing[pass]);
    }
    CHECK(swing[1] > 2.0 * swing[0], "LFO should move the filter (swing %.2f vs %.2f)", swing[1], swing[0]);

    /* 4. stability: random parameters, notes and wheel for a long run */
    srand(1234);
    for (int trial = 0; trial < 300; ++trial) {
        polivoks_bank_init(b);
        for (unsigned p = 0; p < POLIVOKS_NPARAMS; ++p) polivoks_bank_set(b, p, rand() % (parameters[p].max + 1));
        if (trial & 1) { polivoks_bank_set(b, POLIVOKS_RESO, 127); polivoks_bank_set(b, POLIVOKS_CUTOFF, 127); }
        polivoks_bank_mod(b, rand() % 128);
        for (int step = 0; step < 40; ++step) {
            unsigned n = 24 + rand() % 84;
            polivoks_bank_note(b, n, rand() % 2 ? 1 + rand() % 127 : 0, (rand() % 201) - 100);
            int32_t out[POLIVOKS_FRAMES] = {0};
            for (int r = 0; r < 20; ++r) {
                memset(out, 0, sizeof out); polivoks_bank_render(b, out);
                for (unsigned j = 0; j < POLIVOKS_FRAMES; ++j) {
                    CHECK(out[j] <= (int32_t)Q23 && out[j] >= -(int32_t)Q23, "out of range at trial %d", trial);
                    if (failures > 20) goto done;
                }
            }
        }
    }
    printf("stability: 300 random trials done\n");

    /* 5. listening renders */
    struct { const char *name; int wave1, wave2, mode, cutoff, reso, envamt, detune, attack, decay, sustain, release, note, second; } demo[] = {
        {"bass_saw_lp",    1, 1, 0, 30, 70, 90, 70, 2, 45, 20, 30, 36, 0},
        {"screech_bp",     3, 2, 1, 70, 110, 100, 80, 2, 50, 60, 40, 52, 59},
        {"pad_tri_detune", 0, 1, 0, 80, 30, 40, 60, 60, 70, 100, 70, 48, 55},
    };
    for (unsigned d = 0; d < sizeof demo / sizeof demo[0]; ++d) {
        polivoks_bank_init(b);
        polivoks_bank_set(b, POLIVOKS_WAVE1, demo[d].wave1); polivoks_bank_set(b, POLIVOKS_WAVE2, demo[d].wave2);
        polivoks_bank_set(b, POLIVOKS_MODE, demo[d].mode); polivoks_bank_set(b, POLIVOKS_CUTOFF, demo[d].cutoff);
        polivoks_bank_set(b, POLIVOKS_RESO, demo[d].reso); polivoks_bank_set(b, POLIVOKS_ENVAMT, demo[d].envamt);
        polivoks_bank_set(b, POLIVOKS_DETUNE, demo[d].detune); polivoks_bank_set(b, POLIVOKS_ATTACK, demo[d].attack);
        polivoks_bank_set(b, POLIVOKS_DECAY, demo[d].decay); polivoks_bank_set(b, POLIVOKS_SUSTAIN, demo[d].sustain);
        polivoks_bank_set(b, POLIVOKS_RELEASE, demo[d].release);
        polivoks_bank_note(b, demo[d].note, 110, 0); if (demo[d].second) polivoks_bank_note(b, demo[d].second, 110, 0);
        int32_t *x = calloc(400 * POLIVOKS_FRAMES, 4), *y;
        for (int i = 0; i < 400; ++i) { if (i == 250) { polivoks_bank_note(b, demo[d].note, 0, 0); if (demo[d].second) polivoks_bank_note(b, demo[d].second, 0, 0); } polivoks_bank_render(b, x + (size_t)i * POLIVOKS_FRAMES); }
        y = x; double p = 0; for (int i = 0; i < 400 * POLIVOKS_FRAMES; ++i) if (fabs(y[i] / Q23) > p) p = fabs(y[i] / Q23);
        printf("%s: peak %.3f\n", demo[d].name, p);
        CHECK(p > 0.02 && p < 0.99, "%s level %f", demo[d].name, p);
        wav(dir, demo[d].name, x, 400 * POLIVOKS_FRAMES); free(x);
    }
done:
    free(b);
    printf(failures ? "%d FAILURES\n" : "all checks passed\n", failures);
    return failures != 0;
}
