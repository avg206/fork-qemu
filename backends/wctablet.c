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
#include "ui/input.h"


#define DEBUG_WCTABLET_MOUSE

#ifdef DEBUG_WCTABLET_MOUSE
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) \
do {} while (0)
#endif

#define WC_COMMANDS_COUNT 21

#define WC_BUSY_STATE 1
#define WC_BUSY_WITH_CODES 3
#define WC_WAITING_STATE 2
#define WC_OUTPUT_BUF_MAX_LEN 512
#define WC_COMMAND_MAX_LEN 60

#define WC_L7(n) ((n) & 127)
#define WC_M7(n) (((n) >> 7) & 127)
#define WC_H2(n) ((n) >> 14)

#define WC_L4(n) ((n) & 15)
#define WC_H4(n) (((n) >> 4) & 15)

// Avaliable commands
uint8_t wctablet_commands[WC_COMMANDS_COUNT][6] = {
    {0x0a, 0x53, 0x50, 0x0a, 0},         // \nSP\n
    {0x7e, 0x23, 0},                     // ~#
    {0x0a, 0x54, 0x45, 0x0a, 0},         // \nTE\n
    {0x52, 0x45, 0x0a, 0},               // RE\n
    {0x41, 0x53, 0x31, 0x0a, 0},         // AS1\n
    {0x49, 0x43, 0x31, 0x0a, 0},         // IC1\n
    {0x4f, 0x43, 0x31, 0x0a, 0},         // OC1\n
    {0x49, 0x54, 0x88, 0x0d, 0},         // IT3\n
    {0x53, 0x55, 0x88, 0x0d, 0},         // SU3\n
    {0x50, 0x48, 0x31, 0x0a, 0},         // PH1\n
    {0x0d, 0x53, 0x54, 0x0d, 0},         // \rST\n
    {0x0d, 0x53, 0x50, 0x0d, 0},         // \rSP\r
    {0x54, 0x45, 0x0d, 0},               // TE\r
    {0x53, 0x50, 0x0a, 0},               // SP\n
    {0x23, 0x41, 0x4c, 0x31, 0x0d, 0},   // #AL1\r
    {0x53, 0x54, 0x0d, 0},               // ST\n
    {0x0d, 0x54, 0x53, 0x88, 0xd, 0}     // \rTS&\r
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
    "IT3\\n",
    "SU3\\n",
    "PH1\\n",
    "\\rST\\n",
    "\\rSP\\r",
    "TE\\r",
    "SP\\n",
    "#AL1\\r",
    "ST\\n",
    "\\rTS&\\r"
};

// Model string and config string
uint8_t *WC_MODEL_STRING = (uint8_t *) "~#CT-0045R,V1.3-5,";
size_t WC_MODEL_STRING_LENGTH = 18;
uint8_t *WC_CONFIG_STRING = (uint8_t *) "96,N,8,0";
size_t WC_CONFIG_STRING_LENGTH = 8;
uint8_t WC_FULL_CONFIG_STRING[61] = {
    0x5c, 0x39, 0x36, 0x2c, 0x4e, 0x2c, 0x38, 0x2c,
    0x31, 0x28, 0x01, 0x24, 0x57, 0x41, 0x43, 0x30,
    0x30, 0x34, 0x35, 0x5c, 0x5c, 0x50, 0x45, 0x4e, 0x5c,
    0x57, 0x41, 0x43, 0x30, 0x30, 0x30, 0x30, 0x5c,
    0x54, 0x61, 0x62, 0x6c, 0x65, 0x74, 0x0d, 0x0a,
    0x43, 0x54, 0x2d, 0x30, 0x30, 0x34, 0x35, 0x52,
    0x2c, 0x56, 0x31, 0x2e, 0x33, 0x2d, 0x35, 0x0d,
    0x0a, 0x45, 0x37, 0x29
};
size_t WC_FULL_CONFIG_STRING_LENGTH = 61;
int count = 0;
int FIRST_SPEAD_1 = 7000 * 1000;
int FIRST_SPEAD_2 = 8000 * 1000;
int COMMON_SPEAD = 900 * 1000;



// This structure is used to save private info for Wacom Tablet.
typedef struct wctablet_save {
        struct QEMUTimer *transmit_timer;
    /* QEMU timer */
    uint64_t transmit_time;
    /* time to transmit a char in ticks */
    uint8_t query[100];
    int query_index;
    /* Query string from serial */
    uint8_t outbuf[WC_OUTPUT_BUF_MAX_LEN];
    int outlen;
    
    /* Command to be sent to serial port */
    int state;
    /* State of current task */
} wctablet_save;

static int wctablet_memcmp(uint8_t *a1, uint8_t *a2, int count)
{
    int i;

    for (i = 0; i < count; i++) {
        if (a1[i] != a2[i] && a2[i] != 0x88) {
            return -1;
        }
    }
    
    return 0;
}

static int wctablet_check_command(uint8_t *arr, int count)
{
    int i;

    for (i = 0; i < WC_COMMANDS_COUNT; i++) {
        if (wctablet_memcmp(arr, wctablet_commands[i], count) == 0 &&
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
    // uint8_t codes[8] = { 0xa0, 0x0e, 0x06, 0x00, 0x13, 0x3b, 0x00 };
    // uint8_t codes[8] = { 0xe0, 0x05, 0x6a, 0x00, 0x06, 0x64, 0x40 };
    // uint8_t codes[8] = { 0xa0, 0x1c, 0x29, 0x00, 0x19, 0x1c, 0x00 };

    // DPRINTF("x= %d; y= %d; buttons=%x\n", x, y, buttons_state);

    codes[0] = codes[0] | WC_H2(x);
    codes[1] = codes[1] | WC_M7(x);
    codes[2] = codes[2] | WC_L7(x);

    codes[3] = codes[3] | WC_H2(y);
    codes[4] = codes[4] | WC_M7(y);
    codes[5] = codes[5] | WC_L7(y);

    memcpy(save->outbuf + save->outlen, codes, 7);
    save->outlen += 7;
}

static void wctablet_handler(void *opaque)
{
    CharDriverState *chr = (CharDriverState *) opaque;
    wctablet_save *save = (wctablet_save *) chr->opaque;
    int len, i, canWrite;

    canWrite = qemu_chr_be_can_write(chr);
    len = canWrite;
    if (len > save->outlen) {
        len = save->outlen;
    }

    if (len) {
        DPRINTF("-------- Write %2d: ", canWrite);
        for (i = 0; i < len; i++) {
            DPRINTF(" %02x", save->outbuf[i]); 
        }
        DPRINTF("\n");

        qemu_chr_be_write(chr, save->outbuf, len);
        save->outlen -= len;
        if (save->outlen) {
            memmove(save->outbuf, save->outbuf + len, save->outlen);
        }
    }

    timer_mod(save->transmit_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + save->transmit_time);
}

static int wctablet_chr_write (struct CharDriverState *s,
                               const uint8_t *buf, int len)
{
    wctablet_save *save = (wctablet_save *) s->opaque;
    uint8_t c = buf[0];
    uint8_t input;

    save->query[save->query_index++] = c;

    DPRINTF("Receive: %.2x\n", c);

    int comm = wctablet_check_command(save->query, save->query_index);

    if (comm == 1) {
        count++;
    }

    if (comm != -1) {
        if (comm == 1 && count == 2) {
            memcpy(save->outbuf + save->outlen, WC_MODEL_STRING, WC_MODEL_STRING_LENGTH);
            save->outlen += WC_MODEL_STRING_LENGTH;
        }

        if (comm == 3) {
            memcpy(save->outbuf + save->outlen, WC_CONFIG_STRING, WC_CONFIG_STRING_LENGTH);
            save->outlen += WC_CONFIG_STRING_LENGTH;
        }

        // if (comm == 7) {
        //     uint8_t codes[20] = {
        //         0xa1, 0x20, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x18,
        //         0xa1, 0x20, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x82, 0x18
        //     };
        //     memcpy(save->outbuf + save->outlen, codes, 20);
        //     save->outlen += 20;
        // }

        if (comm == 16) {
            input = save->query[3];
            uint8_t codes[7] = {
                0xa3,
                0x88,
                0x88,
                0x03,
                0x7f,
                0x7f,
                0x00
            };
            codes[1] = ((input & 0x80) == 0) ? 0x7e : 0x7f;
            codes[2] = ( ( ( WC_H4(input) & 0x7 ) ^ 0x5) << 4 ) | (WC_L4(input) ^ 0x7);

            memcpy(save->outbuf + save->outlen, codes, 7);
            save->outlen += 7;
            // 
            // if (save->query[3] == 0x5f) {
            //     uint8_t codes[7] = { 0xa3, 0x7e, 0x08, 0x03, 0x7f, 0x7f, 0x00 }; // 0x5f
            //     memcpy(save->outbuf + save->outlen, codes, 7);
            // } else if (save->query[3] == 0x95) {
            //     uint8_t codes[7] = { 0xa3, 0x7f, 0x42, 0x03, 0x7f, 0x7f, 0x00 }; // 0x95
            //     memcpy(save->outbuf + save->outlen, codes, 7);
            // } else if (save->query[3] == 0xcc) {
            //     uint8_t codes[7] = { 0xa3, 0x7f, 0x1b, 0x03, 0x7f, 0x7f, 0x00 }; // 0xcc
            //     memcpy(save->outbuf + save->outlen, codes, 7);
            // } else if (save->query[3] == 0xf1) {
            //     uint8_t codes[7] = { 0xa3, 0x7f, 0x26, 0x03, 0x7f, 0x7f, 0x00 }; // 0xf1
            //     memcpy(save->outbuf + save->outlen, codes, 7);
            // } else { // if (save->query[3] == 0x28) {
            //     uint8_t codes[7] = { 0xa3, 0x7e, 0x7f, 0x03, 0x7f, 0x7f, 0x00 }; // 0x28
            //     memcpy(save->outbuf + save->outlen, codes, 7);
            // }
        }
        

        save->state = WC_BUSY_STATE;

        DPRINTF("-------- Command: %s\n", wctablet_commands_names[comm]);

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
    save->transmit_time = COMMON_SPEAD; // (NANOSECONDS_PER_SECOND / 500) * 10; /* 1200 bauds */

    timer_mod(save->transmit_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + save->transmit_time);


    /* init state machine */
    memcpy(save->outbuf, WC_FULL_CONFIG_STRING, WC_FULL_CONFIG_STRING_LENGTH);
    save->outlen = WC_FULL_CONFIG_STRING_LENGTH;
    save->query_index = 0;
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
