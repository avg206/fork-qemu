/*
 * QEMU Wacome serial tablet emulation
 *
 * Copyright (c) 2008 Lubomir Rintel
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "sysemu/char.h"
#include "ui/console.h"


#define DEBUG_WCTABLET_MOUSE

#ifdef DEBUG_WCTABLET_MOUSE
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "wctablet: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) \
do {} while (0)
#endif

#define WC_COMMANDS_COUNT 20

#define WC_BUSY_STATE 1
#define WC_BUSY_WITH_CODES 3
#define WC_WAITING_STATE 2

#define WC_L7(n) ((n) & 127)
#define WC_M7(n) (((n) >> 7) & 127)
#define WC_H2(n) ((n) >> 14)

// Avaliable commands
uint8_t wctablet_commands[WC_COMMANDS_COUNT][6] = {
    {0x0a, 0x53, 0x50, 0x0a, 0},         // \nSP\n
    {0x7e, 0x23, 0},                     // ~#
    {0x0a, 0x54, 0x45, 0x0a, 0},         // \nTE\n
    {0x52, 0x45, 0x0a, 0},               // RE\n
    {0x41, 0x53, 0x31, 0x0a, 0},         // AS1\n
    {0x49, 0x43, 0x31, 0x0a, 0},         // IC1\n
    {0x4f, 0x43, 0x31, 0x0a, 0},         // OC1\n
    {0x49, 0x54, 0x32, 0x0a, 0},         // IT2\n
    {0x53, 0x55, 0x33, 0x0a, 0},         // SU3\n
    {0x50, 0x48, 0x31, 0x0a, 0},         // PH1\n
    {0x53, 0x54, 0x0a, 0},               // ST\n
    {0x53, 0x50, 0x0d, 0},               // SP\r
    {0x54, 0x45, 0x0d, 0},               // TE\r
    {0x53, 0x50, 0x0a, 0},               // SP\n
    {0x23, 0x41, 0x4c, 0x31, 0x0d, 0},   // #AL1\r
    {0x53, 0x54, 0x0d, 0}                // ST\n
};

// Char strings with avaliable commands
char wctablet_commands_names[WC_COMMANDS_COUNT][7] = {
    "\\nSP\\n",
    "~#",
    "\\nTE\\n",
    "RE\\n",
    "AS1\\n",
    "IC1\\n",
    "OC1\\n",
    "IT2\\n",
    "SU3\\n",
    "PH1\\n",
    "ST\\n",
    "SP\\r",
    "TE\\r",
    "SP\\n",
    "#AL1\\r",
    "ST\\n"
};

// Model string and config string
uint8_t *WC_MODEL_STRING = (uint8_t *) "CT-0045R,V1.3-5";
uint8_t *WC_CONFIG_STRING = (uint8_t *) "96,N,8,0";

// This structure is used to save private info for Wacom Tablet.
typedef struct wctablet_save {
    struct QEMUTimer *transmit_timer;
    /* QEMU timer */
    uint64_t transmit_time;
    /* time to transmit a char in ticks */
    uint8_t query[100];
    int query_index;
    /* Query string from serial */
    uint8_t* command_string;
    uint8_t codes[8];
    int command_length;
    int command_index;
    /* Command to be sent to serial port */
    int state;
    /* State of current task */
} wctablet_save;

static int wctablet_check_command(uint8_t *arr, int count)
{
    int i;

    for (i = 0; i < WC_COMMANDS_COUNT; i++) {
        if (memcmp(arr, wctablet_commands[i], count) == 0 &&
            wctablet_commands[i][count] == 0) {
            return i;
        }
    }

    return -1;
}

static void wctablet_event(void *opaque, int x,
                           int y, int dz, int buttons_state)
{
    CharDriverState *chr = (CharDriverState *) opaque;
    wctablet_save *save = (wctablet_save *) chr->opaque;
    uint8_t codes[8] = { 0xe0, 0, 0, 0, 0, 0, 0 };

    if (save->state == WC_WAITING_STATE) {
        // DPRINTF("x= %d; y= %d; buttons=%x\n", x, y, buttons_state);

        codes[0] = codes[0] | WC_H2(x);
        codes[1] = codes[1] | WC_M7(x);
        codes[2] = codes[2] | WC_L7(x);

        codes[3] = codes[3] | WC_H2(y);
        codes[4] = codes[4] | WC_M7(y);
        codes[5] = codes[5] | WC_L7(y);

        memcpy(save->codes, codes, 7);
        save->command_length = 7;

        save->state = WC_BUSY_WITH_CODES;
        save->command_index = 0;
    }
}

static void wctablet_handler(void *opaque)
{
    CharDriverState *chr = (CharDriverState *) opaque;
    wctablet_save *save = (wctablet_save *) chr->opaque;

    if (qemu_chr_be_can_write(chr) > 0 &&
        save->command_index < save->command_length) {
        int a = save->command_index++;

        if (save->command_index >= save->command_length) {
            save->state = WC_WAITING_STATE;
        }

        uint8_t byte = (save->state == WC_BUSY_WITH_CODES)
            ? (uint8_t) save->codes[a]
            : (uint8_t) save->command_string[a];

        qemu_chr_be_write(chr, &byte, 1);
    }

    timer_mod(save->transmit_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + save->transmit_time);
}

static int wctablet_chr_write (struct CharDriverState *s,
                               const uint8_t *buf, int len)
{
    wctablet_save *save = (wctablet_save *) s->opaque;
    uint8_t c = buf[0];

    if (c == 0x40) {
        return len;
    }

    save->query[save->query_index++] = c;

    // DPRINTF("Receive: %.2x\n", c);

    int comm = wctablet_check_command(save->query, save->query_index);

    if (comm != -1) {
        if (comm == 1) {
            save->command_string = WC_MODEL_STRING;
        }

        if (comm == 3) {
            save->command_string = WC_CONFIG_STRING;
        }

        save->command_length = sizeof(save->command_string);

        save->state = WC_BUSY_STATE;
        save->command_index = 0;

        // DPRINTF("-------- Command: %s\n", wctablet_commands_names[comm]);
        save->query_index = 0;
    }

    return len;
}

static void wctablet_chr_close (struct CharDriverState *chr)
{
    g_free (chr->opaque);
    g_free (chr);
}

static CharDriverState *qemu_chr_open_wctablet(const char *id,
                                              ChardevBackend *backend,
                                              ChardevReturn *ret,
                                              Error **errp)
{
    ChardevCommon *common = backend->u.wctablet.data;
    CharDriverState *chr;
    wctablet_save *save;

    chr = qemu_chr_alloc(common, errp);
    save = g_malloc0(sizeof(wctablet_save));
    if (!chr) {
        return NULL;
    }
    chr->chr_write = wctablet_chr_write;
    chr->chr_close = wctablet_chr_close;
    chr->explicit_be_open = true;

    /* create a new QEMU's timer with wctablet_handler() as timeout handler. */
    save->transmit_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                       (QEMUTimerCB *) wctablet_handler, chr);

    /* calculate the transmit_time for 1200 bauds transmission */
    save->transmit_time = (NANOSECONDS_PER_SECOND / 500) * 10; /* 1200 bauds */

    timer_mod(save->transmit_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + save->transmit_time);


    /* init state machine */
    save->query_index = 0;
    save->command_index = 1;
    save->command_length = 0;
    save->command_string = (uint8_t *) "";
    save->state = WC_BUSY_STATE;
    /* keep address of wctablet_save */

    chr->opaque = save;

    qemu_add_mouse_event_handler(wctablet_event, chr, 1,
                                 "QEMU Wacome Pen Tablet");

    return chr;
}

static void register_types(void)
{
    register_char_driver("wctablet", CHARDEV_BACKEND_KIND_WCTABLET, NULL,
                         qemu_chr_open_wctablet);
}

type_init(register_types);
