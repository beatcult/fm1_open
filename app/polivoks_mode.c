/* SPDX-License-Identifier: GPL-3.0-only WITH GPL-3.0-linking-exception */
#include "polivoks_mode.h"
#include "fm1_device.h"
#include "instrument.h"

enum { EV_NOTE, EV_PEDAL, EV_BEND, EV_MOD, EV_PARAM, EV_PANIC };
typedef struct { uint8_t type, a, b, pad; int16_t value; } event_t;
enum { RING = 64 };

static event_t ring[RING];
static volatile uint8_t head, tail;                 /* head: producers (IRQs off); tail: audio interrupt */
static volatile uint8_t mode;
static uint32_t storage[POLIVOKS_BANK_BYTES / 4];
static uint8_t shadow[POLIVOKS_NPARAMS], shadow_ready;  /* control-side copy for the panel */
static int32_t mono[POLIVOKS_FRAMES];

unsigned polivoks_mode_enabled(void) { return mode; }

static void ensure_shadow(void)
{
    if (shadow_ready) return;
    for (unsigned p = 0; p < POLIVOKS_NPARAMS; ++p) shadow[p] = polivoks_param((int)p)->def;
    shadow_ready = 1;
}

static void push(uint8_t type, uint8_t a, uint8_t b, int16_t value)
{
    uint32_t irq = fm1_device_irq_save();
    uint8_t next = (uint8_t)((head + 1U) % RING);
    if (next != tail) {                              /* full: drop the new event, never block */
        ring[head].type = type; ring[head].a = a; ring[head].b = b; ring[head].value = value;
        head = next;
    }
    fm1_device_irq_restore(irq);
}
void polivoks_mode_set(unsigned on)
{
    on = on != 0;
    if (on == mode) return;
    if (on) {
        /* Initialise while the audio interrupt is excluded, then publish. */
        uint32_t irq = fm1_device_irq_save();
        polivoks_bank_t *bank = (polivoks_bank_t *)storage;
        ensure_shadow();
        polivoks_bank_init(bank);
        for (unsigned p = 0; p < POLIVOKS_NPARAMS; ++p) polivoks_bank_set(bank, p, shadow[p]);
        head = tail = 0;
        fm1_device_irq_restore(irq);
        groove_midi_panic();                         /* release anything the engine is holding */
        mode = 1;
    } else {
        mode = 0;
        groove_midi_panic();
    }
}
void polivoks_param_set(unsigned parameter, unsigned value)
{
    const fm1_param_t *d = polivoks_param((int)parameter);
    if (!d) return;
    if (value > d->max) value = d->max;
    ensure_shadow();
    shadow[parameter] = (uint8_t)value;
    if (mode) push(EV_PARAM, (uint8_t)parameter, (uint8_t)value, 0);
}
unsigned polivoks_param_get(unsigned parameter)
{
    const fm1_param_t *d = polivoks_param((int)parameter);
    if (!d) return 0;
    ensure_shadow();
    return shadow[parameter];
}
int polivoks_midi_filter(const uint8_t *m, size_t n)
{
    if (!mode || !m || n < 2) return 0;
    switch (m[0] & 0xf0U) {
    case 0x90: if (n < 3) return 0; push(EV_NOTE, m[1], m[2], 0); return 1;
    case 0x80: push(EV_NOTE, m[1], 0, 0); return 1;
    case 0xe0:
        if (n < 3) return 0;
        push(EV_BEND, 0, 0, (int16_t)((int)m[1] + ((int)m[2] << 7) - 8192)); return 1;
    case 0xb0:
        if (n < 3) return 0;
        if (m[1] == 1) { push(EV_MOD, m[2], 0, 0); return 1; }
        if (m[1] == 64) { push(EV_PEDAL, m[2] >= 64, 0, 0); return 1; }
        if (m[1] == 120 || m[1] == 123) { push(EV_PANIC, 0, 0, 0); return 0; }  /* engine gets it too */
        return 0;
    default: return 0;
    }
}

FM1_AUDIO_CODE void polivoks_audio_mix(int32_t *output)
{
    if (!mode || !output) return;
    polivoks_bank_t *bank = (polivoks_bank_t *)storage;
    while (tail != head) {
        const event_t *e = &ring[tail];
        switch (e->type) {
        case EV_NOTE: polivoks_bank_note(bank, e->a, e->b, 0); break;
        case EV_PEDAL: polivoks_bank_pedal(bank, e->a); break;
        case EV_BEND: polivoks_bank_bend(bank, (int32_t)e->value * 16); break;  /* +-2 semitones */
        case EV_MOD: polivoks_bank_mod(bank, e->a); break;
        case EV_PARAM: polivoks_bank_set(bank, e->a, e->b); break;
        default: polivoks_bank_panic(bank); break;
        }
        tail = (uint8_t)((tail + 1U) % RING);
    }
    for (unsigned i = 0; i < POLIVOKS_FRAMES; ++i) mono[i] = 0;
    if (!polivoks_bank_render(bank, mono)) return;
    for (unsigned i = 0; i < POLIVOKS_FRAMES; ++i) {
        for (unsigned c = 0; c < 2; ++c) {
            int32_t v = output[2 * i + c] + mono[i];
            output[2 * i + c] = v > 8388607 ? 8388607 : v < -8388608 ? -8388608 : v;
        }
    }
}
