// SPDX-License-Identifier: GPL-2.0+

#include <cpu_func.h>
#include <debug_uart.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <video_font_8x16.h>

#define FB		0xfac00000UL
#define FB_W		1080
#define FB_H		2400
#define FB_STRIDE	4320
#define SCALE		2
#define CW		(8 * SCALE)
#define CH		(16 * SCALE)
#define COLS		(FB_W / CW)
#define ROWS		(FB_H / CH)

#define DECON0		0x1c300000
#define DECON_TRIG_CON	0x30
#define DECON_TRIG_HW	(BIT(13) | BIT(12) | BIT(6) | BIT(5) | BIT(0))
#define DECON_SHD_UP	0x50
#define DECON_SHD_ALL	(BIT(31) | BIT(20) | GENMASK(5, 0))

static int fb_x __section(".data");
static int fb_y __section(".data");

static void fb_kick(void)
{
	writel(DECON_TRIG_HW, DECON0 + DECON_TRIG_CON);
	writel(DECON_SHD_ALL, DECON0 + DECON_SHD_UP);
}

static void fb_flush_row(int row)
{
	ulong start = FB + (ulong)row * CH * FB_STRIDE;

	flush_dcache_range(start, start + (ulong)CH * FB_STRIDE);
}

static void fb_clear_row(int row)
{
	u32 *p = (u32 *)(FB + (ulong)row * CH * FB_STRIDE);
	int i;

	for (i = 0; i < CH * FB_STRIDE / 4; i++)
		p[i] = 0xff000000;
}

static void fb_glyph(int col, int row, unsigned char c)
{
	const unsigned char *g = &video_fontdata_8x16[c * 16];
	int x, y, sx, sy;

	for (y = 0; y < 16; y++)
		for (sy = 0; sy < SCALE; sy++) {
			u32 *p = (u32 *)(FB + (ulong)(row * CH + y * SCALE + sy) *
					 FB_STRIDE) + col * CW;

			for (x = 0; x < 8; x++)
				for (sx = 0; sx < SCALE; sx++)
					*p++ = (g[y] & (0x80 >> x)) ?
						0xffffffff : 0xff000000;
		}
}

static void fb_newline(void)
{
	fb_flush_row(fb_y);
	fb_x = 0;
	if (++fb_y >= ROWS)
		fb_y = 0;
	fb_clear_row(fb_y);
	fb_flush_row(fb_y);
	fb_kick();
}

static inline void _debug_uart_init(void)
{
	int r;

	fb_x = 0;
	fb_y = 0;
	for (r = 0; r < ROWS; r++)
		fb_clear_row(r);
	flush_dcache_range(FB, FB + (ulong)FB_H * FB_STRIDE);
	fb_kick();
}

static inline void _debug_uart_putc(int ch)
{
	if (ch == '\r')
		return;
	if (ch == '\n') {
		fb_newline();
		return;
	}
	fb_glyph(fb_x, fb_y, ch);
	if (++fb_x >= COLS)
		fb_newline();
}

DEBUG_UART_FUNCS
