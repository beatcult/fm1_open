/* SPDX-License-Identifier: GPL-3.0-only WITH GPL-3.0-linking-exception */
/* Formanta Polivoks-style duophonic voice bank for the FM-1.
 *
 * Two independent voices, each with two oscillators (triangle, saw, square,
 * narrow pulse, noise), a nonlinear 12 dB resonant filter that switches between
 * low-pass and band-pass, and an ADSR that shapes both amplitude and filter.
 * A slow LFO (mod wheel / CC1) sweeps pitch and cutoff. Pitch drifts like the
 * original's free-running analog oscillators.
 *
 * Output is mono signed Q23 (full scale 8388608), added to the engine's mix
 * by the application. Everything below is audio-interrupt safe: no libc, no
 * allocation, no flash access. Control-side calls (set/note/...) must be made
 * with the audio interrupt held off by the caller, or from the audio owner. */
#ifndef FM1_POLIVOKS_H
#define FM1_POLIVOKS_H
#include <stddef.h>
#include <stdint.h>
#include "fm1_param.h"

#define POLIVOKS_FRAMES 128
#define POLIVOKS_VOICES 2

/* Parameter pages of four, in KNOB 1-4 order. */
enum {
    POLIVOKS_CUTOFF, POLIVOKS_RESO, POLIVOKS_MODE, POLIVOKS_ENVAMT,    /* page 1 */
    POLIVOKS_WAVE1, POLIVOKS_WAVE2, POLIVOKS_DETUNE, POLIVOKS_GLIDE,   /* page 2 */
    POLIVOKS_ATTACK, POLIVOKS_DECAY, POLIVOKS_SUSTAIN, POLIVOKS_RELEASE, /* page 3 */
    POLIVOKS_NPARAMS
};
enum { POLIVOKS_MODE_LP, POLIVOKS_MODE_BP };
enum { POLIVOKS_WAVE_TRI, POLIVOKS_WAVE_SAW, POLIVOKS_WAVE_SQR, POLIVOKS_WAVE_PULSE, POLIVOKS_WAVE_NOISE };

/* Opaque; storage must be at least this large and 4-byte aligned. */
#define POLIVOKS_BANK_BYTES 384
typedef struct polivoks_bank polivoks_bank_t;
size_t polivoks_bank_size(void);
/* Caller owns 4-byte-aligned storage of polivoks_bank_size() bytes. */
void polivoks_bank_init(polivoks_bank_t *b);
void polivoks_bank_panic(polivoks_bank_t *b);              /* silence now, keep parameters */
/* velocity 0 is note-off. cents: fine tune, -100..100. */
void polivoks_bank_note(polivoks_bank_t *b, unsigned note, unsigned velocity, int cents);
void polivoks_bank_pedal(polivoks_bank_t *b, unsigned down);
void polivoks_bank_bend(polivoks_bank_t *b, int32_t semitones_q16);
void polivoks_bank_mod(polivoks_bank_t *b, unsigned value); /* 0..127, CC1 */
/* Adds POLIVOKS_FRAMES mono Q23 samples to out. Returns 0 when silent. */
unsigned polivoks_bank_render(polivoks_bank_t *b, int32_t *out);
void polivoks_bank_set(polivoks_bank_t *b, unsigned parameter, unsigned value);
unsigned polivoks_bank_get(const polivoks_bank_t *b, unsigned parameter);
const fm1_param_t *polivoks_param(int parameter);
unsigned polivoks_bank_active(const polivoks_bank_t *b);
#endif
