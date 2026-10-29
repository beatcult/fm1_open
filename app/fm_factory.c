/* SPDX-License-Identifier: GPL-3.0-only WITH GPL-3.0-linking-exception */
#include "fm_patch.h"

/* Polivoks build: the 128-voice factory DX7 table (about 10 KB of flash and a
 * 16 KB SRAM expansion buffer) was removed to make room for the Polivoks voice.
 * Every factory program now returns the same single voice (the original
 * program 0), kept in SRAM so the audio path never reads Flash. Banks saved to
 * the voice library or projects still take priority over it. */
static const uint8_t factory_voice[128] FM1_DSP_DATA = {
    95,29,20,50,99,95,0,0,41,0,19,0,115,24,79,2,0,95,20,20,50,99,95,0,0,0,0,0,0,3,0,99,2,0,95,29,20,50,99,95,0,0,0,0,0,0,59,24,89,2,0,95,20,20,50,99,95,0,0,0,0,0,0,59,8,99,2,0,95,50,35,78,99,75,0,0,0,0,0,0,59,28,58,28,0,96,25,25,67,99,75,0,0,0,0,0,0,83,8,99,2,0,94,67,95,60,50,50,50,50,4,6,15,33,0,0,40,24,80,73,65,78,79,32,49,32,32,32
};
FM1_DSP_CODE const uint8_t *groove_fm_factory_raw(unsigned program)
{
    (void)program;
    return factory_voice;
}
FM1_DSP_CODE void groove_fm_factory(uint8_t p[156], unsigned program)
{
    groove_fm_unpack(p,groove_fm_factory_raw(program));
}
