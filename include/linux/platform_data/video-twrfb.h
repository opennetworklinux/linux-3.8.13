/*
 * Copyright 2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Freescale TFT LCD Platform data for SSD1289 framebuffer driver
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

/* LCD description */
struct fsl_ssd1289_fb_display {
	/* Screen size */
	unsigned short width;
	unsigned short height;

	/* Screen info */
	unsigned short xres;
	unsigned short yres;
	unsigned short bpp;
};
