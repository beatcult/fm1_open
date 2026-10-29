/* SPDX-License-Identifier: GPL-3.0-only WITH GPL-3.0-linking-exception */
/* Formanta Polivoks-style duophonic voice bank. See polivoks.h.
 *
 * Fixed 44.1 kHz, 128-frame blocks, float DSP, no libc: this code runs in the
 * audio interrupt and must stay in internal RAM so it keeps running while
 * Flash is being written (hence the code/const segments below). */
#include "polivoks.h"
#ifndef FM1_HOST_TEST
#pragma code_seg(".volatile_ram_code")
#pragma const_seg(".volatile_ram.fm1")
#endif

#define SR 44100.0f
#define CHUNK 16                /* control-rate update interval, in samples */
#define HELD_MAX 8
#define Q23 8388608.0f

typedef struct {
    uint8_t note, gate, stage, pad;   /* stage: 0 idle, 1 attack, 2 decay/sustain, 3 release */
    float pitch, target;              /* semitones (MIDI note scale, fractional) */
    float ph1, ph2, drift1, drift2, dt1, dt2;
    float env, vel;
    float s1, s2;                     /* filter integrator states */
} pv_voice_t;

struct polivoks_bank {
    uint8_t pot[POLIVOKS_NPARAMS];
    uint8_t pedal, wheel, nheld, pad;
    uint8_t held[HELD_MAX], sustained[HELD_MAX];
    int8_t heldcents[HELD_MAX];
    pv_voice_t v[POLIVOKS_VOICES];
    uint32_t rng, chunks;
    float lfo_phase;
    int32_t bend;                     /* Q16 semitones */
    /* derived from pot[] */
    float cutoff, k, envamt, glide, att, dec, sus, rel, detune;
};

static const char *const wave_names[] = {"Tri", "Saw", "Sqr", "Pulse", "Noise"};
static const char *const mode_names[] = {"LP", "BP"};
static const fm1_param_t parameters[POLIVOKS_NPARAMS] = {
    {"Cutoff", 127, 60, 0}, {"Reso", 127, 40, 0}, {"Mode", 1, 0, mode_names}, {"EnvAmt", 127, 50, 0},
    {"Wave1", 4, 1, wave_names}, {"Wave2", 4, 1, wave_names}, {"Detune", 127, 66, 0}, {"Glide", 127, 0, 0},
    {"Attack", 127, 4, 0}, {"Decay", 127, 50, 0}, {"Sustain", 127, 80, 0}, {"Release", 127, 36, 0}};

const fm1_param_t *polivoks_param(int p) { return (unsigned)p < POLIVOKS_NPARAMS ? parameters + p : 0; }
size_t polivoks_bank_size(void) { return sizeof(struct polivoks_bank); }

/* ---- math helpers (Taylor/Pade; no libm) ---- */
static inline float clampf(float x, float a, float b) { return x < a ? a : x > b ? b : x; }
static inline float absf(float x) { return x < 0 ? -x : x; }
static float pv_exp2(float x)
{
    x = clampf(x, -120.0f, 120.0f);
    int n = (int)(x + (x < 0 ? -0.5f : 0.5f));
    float a = (x - (float)n) * 0.6931471805599453f;
    float p = 1.0f + a * (1.0f + a * (0.5f + a * (1.0f / 6.0f + a * (1.0f / 24.0f + a * (1.0f / 120.0f + a / 720.0f)))));
    union { uint32_t u; float f; } s;
    s.u = (uint32_t)(n + 127) << 23;
    return p * s.f;
}
static inline float pv_exp(float x) { return pv_exp2(x * 1.4426950408889634f); }
/* Smooth saturator standing in for the OTA/multiplier core's soft clipping. */
static inline float sat(float x)
{
    if (x > 3.0f) return 1.0f;
    if (x < -3.0f) return -1.0f;
    return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}
/* tan(pi*fc/SR) for the filter coefficient; Pade, valid to ~fc = 16 kHz. */
static inline float warp(float fc)
{
    float x = 3.14159265f * fc * (1.0f / SR), x2 = x * x;
    return x * (105.0f - 10.0f * x2) / (105.0f - 45.0f * x2 + x2 * x2);
}
static inline float noise(uint32_t *s)
{
    uint32_t n = *s; n ^= n << 13; n ^= n >> 17; n ^= n << 5; *s = n;
    return (float)(n >> 8) * (1.0f / 8388608.0f) - 1.0f;
}
static inline float blep(float t, float dt)
{
    if (t < dt) { float x = t / dt; return x + x - x * x - 1.0f; }
    if (t > 1.0f - dt) { float x = (t - 1.0f) / dt; return x * x + x + x + 1.0f; }
    return 0.0f;
}
static inline float wrap1(float p) { return p >= 1.0f ? p - 1.0f : p; }

static float osc(float *phase, float inc, unsigned wave, uint32_t *rng)
{
    float p = *phase, v;
    switch (wave) {
    case POLIVOKS_WAVE_TRI: v = 4.0f * absf(p - 0.5f) - 1.0f; break;
    case POLIVOKS_WAVE_SAW: v = 2.0f * p - 1.0f - blep(p, inc); break;
    case POLIVOKS_WAVE_SQR:
        v = (p < 0.5f ? 1.0f : -1.0f) + blep(p, inc) - blep(wrap1(p + 0.5f), inc); break;
    case POLIVOKS_WAVE_PULSE:
        v = (p < 0.25f ? 1.0f : -1.0f) + blep(p, inc) - blep(wrap1(p + 0.75f), inc); break;
    default: v = noise(rng); break;
    }
    p += inc; if (p >= 1.0f) p -= 1.0f;
    *phase = p;
    return v;
}

/* ---- parameters ---- */
static void derive(polivoks_bank_t *b)
{
    const uint8_t *p = b->pot;
    float u;
    b->cutoff = 40.0f * pv_exp2(p[POLIVOKS_CUTOFF] * (1.0f / 127.0f) * 9.0f);
    u = p[POLIVOKS_RESO] * (1.0f / 127.0f);
    b->k = 2.0f * (1.0f - u * 0.985f) * (1.0f - u * 0.5f * u) + 0.03f;   /* ~2 .. ~0.04 */
    b->envamt = p[POLIVOKS_ENVAMT] * (1.0f / 127.0f) * 6.0f;
    u = (p[POLIVOKS_DETUNE] - 64) * (1.0f / 63.0f);
    b->detune = (u < 0 ? -1.0f : 1.0f) * u * u * 12.0f;                  /* +-12 semitones, fine near centre */
    u = p[POLIVOKS_GLIDE] * (1.0f / 127.0f);
    /* per-chunk glide coefficient; 0 = instant */
    b->glide = u <= 0.0f ? 1.0f : 1.0f - pv_exp(-(float)CHUNK / (SR * 0.002f * pv_exp2(u * 10.0f)));
    b->att = 1.0f - pv_exp(-1.0f / (SR * 0.001f * pv_exp2(p[POLIVOKS_ATTACK] * (1.0f / 127.0f) * 12.2f)));
    b->dec = 1.0f - pv_exp(-1.0f / (SR * 0.005f * pv_exp2(p[POLIVOKS_DECAY] * (1.0f / 127.0f) * 10.2f)));
    b->rel = 1.0f - pv_exp(-1.0f / (SR * 0.005f * pv_exp2(p[POLIVOKS_RELEASE] * (1.0f / 127.0f) * 10.2f)));
    b->sus = p[POLIVOKS_SUSTAIN] * (1.0f / 127.0f);
}
void polivoks_bank_set(polivoks_bank_t *b, unsigned parameter, unsigned value)
{
    if (parameter >= POLIVOKS_NPARAMS) return;
    if (value > parameters[parameter].max) value = parameters[parameter].max;
    b->pot[parameter] = (uint8_t)value;
    derive(b);
}
unsigned polivoks_bank_get(const polivoks_bank_t *b, unsigned parameter)
{
    return parameter < POLIVOKS_NPARAMS ? b->pot[parameter] : 0;
}

void polivoks_bank_init(polivoks_bank_t *b)
{
    uint8_t *raw = (uint8_t *)b;
    for (size_t i = 0; i < sizeof *b; ++i) raw[i] = 0;
    for (unsigned i = 0; i < POLIVOKS_NPARAMS; ++i) b->pot[i] = parameters[i].def;
    b->rng = 0x2545f491u;
    derive(b);
}
void polivoks_bank_panic(polivoks_bank_t *b)
{
    for (unsigned i = 0; i < POLIVOKS_VOICES; ++i) {
        pv_voice_t *v = &b->v[i];
        v->gate = v->stage = 0; v->env = 0; v->s1 = v->s2 = 0;
    }
    b->nheld = 0; b->pedal = 0;
}

/* ---- note handling: the two most recently held notes sound ---- */
static void assign(polivoks_bank_t *b)
{
    int want[POLIVOKS_VOICES], taken[POLIVOKS_VOICES] = {0, 0};
    for (unsigned i = 0; i < POLIVOKS_VOICES; ++i)
        want[i] = b->nheld > i ? (int)(b->nheld - 1 - i) : -1;      /* indexes into held[] */
    for (unsigned i = 0; i < POLIVOKS_VOICES; ++i) {
        pv_voice_t *v = &b->v[i];
        if (!v->gate) continue;
        int keep = 0;
        for (unsigned w = 0; w < POLIVOKS_VOICES; ++w)
            if (want[w] >= 0 && !taken[w] && b->held[want[w]] == v->note) { taken[w] = 1; keep = 1; break; }
        if (!keep) { v->gate = 0; v->stage = 3; }
    }
    for (unsigned w = 0; w < POLIVOKS_VOICES; ++w) {
        if (want[w] < 0 || taken[w]) continue;
        pv_voice_t *free_voice = 0;
        for (unsigned i = 0; i < POLIVOKS_VOICES; ++i) {
            pv_voice_t *v = &b->v[i];
            if (v->gate) continue;
            if (!free_voice || v->env < free_voice->env) free_voice = v;
        }
        if (!free_voice) break;
        pv_voice_t *v = free_voice;
        float target = (float)b->held[want[w]] + b->heldcents[want[w]] * 0.01f;
        if (v->stage == 0) { v->pitch = target; v->s1 = v->s2 = 0; }  /* from silence: no glide, clean filter */
        v->target = target; v->note = b->held[want[w]];
        v->gate = 1; v->stage = 1;                                   /* attack continues from the current level */
    }
}
void polivoks_bank_note(polivoks_bank_t *b, unsigned note, unsigned velocity, int cents)
{
    if (note > 127) return;
    unsigned i, found = HELD_MAX;
    for (i = 0; i < b->nheld; ++i) if (b->held[i] == note) { found = i; break; }
    if (velocity) {
        if (found != HELD_MAX) {                                     /* re-press: move to the top */
            for (i = found; i + 1 < b->nheld; ++i) {
                b->held[i] = b->held[i + 1]; b->sustained[i] = b->sustained[i + 1]; b->heldcents[i] = b->heldcents[i + 1];
            }
            --b->nheld;
        } else if (b->nheld == HELD_MAX) {                           /* full: forget the oldest */
            for (i = 0; i + 1 < b->nheld; ++i) {
                b->held[i] = b->held[i + 1]; b->sustained[i] = b->sustained[i + 1]; b->heldcents[i] = b->heldcents[i + 1];
            }
            --b->nheld;
        }
        if (cents < -100) cents = -100;
        if (cents > 100) cents = 100;
        b->held[b->nheld] = (uint8_t)note; b->sustained[b->nheld] = 0; b->heldcents[b->nheld] = (int8_t)cents;
        ++b->nheld;
        for (i = 0; i < POLIVOKS_VOICES; ++i)                        /* velocity of the newest note */
            if (!b->v[i].gate || b->v[i].note == note) b->v[i].vel = velocity * (1.0f / 127.0f);
    } else if (found != HELD_MAX) {
        if (b->pedal) { b->sustained[found] = 1; return; }
        for (i = found; i + 1 < b->nheld; ++i) {
            b->held[i] = b->held[i + 1]; b->sustained[i] = b->sustained[i + 1]; b->heldcents[i] = b->heldcents[i + 1];
        }
        --b->nheld;
    }
    assign(b);
}
void polivoks_bank_pedal(polivoks_bank_t *b, unsigned down)
{
    b->pedal = down ? 1 : 0;
    if (down) return;
    unsigned out = 0;
    for (unsigned i = 0; i < b->nheld; ++i)
        if (!b->sustained[i]) {
            b->held[out] = b->held[i]; b->sustained[out] = 0; b->heldcents[out] = b->heldcents[i]; ++out;
        }
    b->nheld = (uint8_t)out;
    assign(b);
}
void polivoks_bank_bend(polivoks_bank_t *b, int32_t semitones_q16) { b->bend = semitones_q16; }
void polivoks_bank_mod(polivoks_bank_t *b, unsigned value) { b->wheel = (uint8_t)(value > 127 ? 127 : value); }
unsigned polivoks_bank_active(const polivoks_bank_t *b)
{
    unsigned n = 0;
    for (unsigned i = 0; i < POLIVOKS_VOICES; ++i) n += b->v[i].stage != 0;
    return n;
}

/* ---- rendering ---- */
unsigned polivoks_bank_render(polivoks_bank_t *b, int32_t *out)
{
    if (!polivoks_bank_active(b)) return 0;
    const unsigned wave1 = b->pot[POLIVOKS_WAVE1], wave2 = b->pot[POLIVOKS_WAVE2], bandpass = b->pot[POLIVOKS_MODE];
    const float depth = b->wheel * (1.0f / 127.0f), bend = (float)b->bend * (1.0f / 65536.0f);
    float acc[POLIVOKS_FRAMES];
    for (unsigned i = 0; i < POLIVOKS_FRAMES; ++i) acc[i] = 0.0f;

    for (unsigned c = 0; c < POLIVOKS_FRAMES / CHUNK; ++c) {
        /* shared slow LFO, ~5.3 Hz triangle */
        b->lfo_phase += CHUNK * 5.3f / SR; if (b->lfo_phase >= 1.0f) b->lfo_phase -= 1.0f;
        float lfo = 4.0f * absf(b->lfo_phase - 0.5f) - 1.0f;
        ++b->chunks;
        for (unsigned vi = 0; vi < POLIVOKS_VOICES; ++vi) {
            pv_voice_t *v = &b->v[vi];
            if (!v->stage) continue;
            /* pitch: glide, bend, vibrato, analog-style drift (random walk, a few cents) */
            v->pitch += (v->target - v->pitch) * b->glide;
            if (!((b->chunks + vi * 7) & 31)) { v->dt1 = noise(&b->rng) * 7.0f; v->dt2 = noise(&b->rng) * 7.0f; }
            v->drift1 += (v->dt1 - v->drift1) * 0.02f;
            v->drift2 += (v->dt2 - v->drift2) * 0.02f;
            float semis = v->pitch + bend + lfo * depth * 0.3f;
            float inc1 = 8.175799f * pv_exp2((semis + v->drift1 * 0.01f) * (1.0f / 12.0f)) * (1.0f / SR);
            float inc2 = 8.175799f * pv_exp2((semis + b->detune + v->drift2 * 0.01f) * (1.0f / 12.0f)) * (1.0f / SR);
            inc1 = clampf(inc1, 1e-5f, 0.45f); inc2 = clampf(inc2, 1e-5f, 0.45f);
            /* filter coefficients from the current envelope and LFO */
            float fc = b->cutoff * pv_exp2(b->envamt * v->env + lfo * depth * 1.5f);
            float g = warp(clampf(fc, 20.0f, 16000.0f)), k = b->k;
            float h = 1.0f / (1.0f + k * g + g * g);
            float level = (0.6f + 0.4f * v->vel) * 0.5f * Q23 * 0.6f;
            for (unsigned i = 0; i < CHUNK; ++i) {
                /* ADSR, RC-style exponential segments; one envelope shapes amp and filter */
                switch (v->stage) {
                case 1: v->env += (1.15f - v->env) * b->att; if (v->env >= 1.0f) { v->env = 1.0f; v->stage = 2; } break;
                case 2: v->env += (b->sus - v->env) * b->dec; break;
                default: v->env -= v->env * b->rel; if (v->env < 2e-4f) { v->env = 0; v->stage = 0; } break;
                }
                float in = 0.5f * (osc(&v->ph1, inc1, wave1, &b->rng) + osc(&v->ph2, inc2, wave2, &b->rng));
                in = sat(in * 1.4f);
                /* two-integrator state-variable filter; BP integrator saturates like the original's core */
                float hp = (in - (k + g) * v->s1 - v->s2) * h;
                float bp = g * hp + v->s1;
                v->s1 = sat(bp + g * hp);
                float lp = g * bp + v->s2;
                v->s2 = lp + g * bp;
                float y = bandpass ? bp * (k + 0.5f) * 1.6f : lp;
                acc[c * CHUNK + i] += y * v->env * level;
            }
            if (!v->stage) { v->s1 = v->s2 = 0.0f; }
        }
    }
    for (unsigned i = 0; i < POLIVOKS_FRAMES; ++i) {
        float s = clampf(acc[i], -Q23, Q23 - 1.0f);
        out[i] += (int32_t)s;
    }
    return 1;
}
