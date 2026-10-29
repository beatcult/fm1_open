/* SPDX-License-Identifier: GPL-3.0-only WITH GPL-3.0-linking-exception */
/* Polivoks mode: the application renders a Polivoks voice bank itself and mixes
 * it into the output block next to M-VAVE's engine. While the mode is on, note,
 * pitch-bend, mod-wheel and pedal messages (panel keys and MIDI input) go to
 * the Polivoks instead of the engine. All other messages still reach the engine. */
#ifndef FM1_POLIVOKS_MODE_H
#define FM1_POLIVOKS_MODE_H
#include <stddef.h>
#include <stdint.h>
#include "fm1.h"
#include "polivoks.h"

/* Control/routing tasks. */
unsigned polivoks_mode_enabled(void);
void polivoks_mode_set(unsigned on);
/* Returns 1 when the message was consumed by the Polivoks, 0 to pass it on. */
int polivoks_midi_filter(const uint8_t *message, size_t bytes);
/* Parameter access for the panel; the audio side applies changes at the next block. */
void polivoks_param_set(unsigned parameter, unsigned value);
unsigned polivoks_param_get(unsigned parameter);
/* Audio interrupt only: add the Polivoks to a stereo Q23 block. */
FM1_AUDIO_CODE void polivoks_audio_mix(int32_t *output);
#endif
