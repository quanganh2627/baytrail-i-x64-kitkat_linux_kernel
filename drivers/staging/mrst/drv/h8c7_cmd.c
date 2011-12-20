/*
 * Copyright (c)  2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicensen
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 * Thomas Eaton <thomas.g.eaton@intel.com>
 * Scott Rowe <scott.m.rowe@intel.com>
 * Jim Liu <jim.liu@intel.com>
*/

#include "displays/h8c7_cmd.h"
#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dbi_dpu.h"
#include "mdfld_dsi_pkg_sender.h"

/* ************************************************************************* *\
 * FUNCTION: mdfld_h8c7_dbi_ic_init
 *
 * DESCRIPTION:  This function is called only by mrst_dsi_mode_set and
 *               restore_display_registers.  since this function does not
 *               acquire the mutex, it is important that the calling function
 *               does!
\* ************************************************************************* */
static u32 h8c7_exit_sleep_mode[] = {0x00000011};
static u32 h8c7_mcs_protect_off[] = {0x9283ffb9};
static u32 h8c7_set_tear_on[] = {0x00000035};
static u32 h8c7_set_full_brightness[] = {0x0000ff51};
static u32 h8c7_turn_on_backlight[] = {0x00002453};
static u32 h8c7_disable_cabc[] = {0x00000055};
static u32 h8c7_ic_bias_current[] = {0x826005bf, 0x00000000};
static u32 h8c7_set_power[] = {0x44007cb1, 0x0d0d0024, 0x3f3f1a12, 0x00007242};
static u32 h8c7_set_disp_reg[] = {0x05c80fb2, 0x0084080f, 0x040f05ff, 0x00000020};
static u32 h8c7_set_command_cyc[] = {0x050000b4, 0x1605a000, 0x1603309d,
	0x00030300, 0x0707061b, 0x00000000};
static u32 h8c7_set_mipi_ctrl[] = {0x008312ba};
static u32 h8c7_command_mode[] = {0x000008c2};
static u32 h8c7_set_blanking_opt_2[] = {0x004000c7};
static u32 h8c7_set_panel[] = {0x000008cc};
static u32 h8c7_set_eq_func_ltps[] = {0x00000cd4};
static u32 h8c7_set_ltps_ctrl_output[] = {0x080800d5, 0x66554400, 0xcccccc77,
	0x667700cc, 0xcccc4455, 0x0000cccc};
static u32 h8c7_set_video_cyc[] = {0x040000d8, 0x1604a000, 0x1603309d,
	0x00030300, 0x0707061b, 0x00000000};
static u32 h8c7_gamma_r[] = {0x3c3e3ae0, 0x3332312f, 0x0c080446, 0x110f100d,
	0x3e3a1710, 0x32312f3c, 0x08044633, 0x0f100d0c, 0x00171011};
static u32 h8c7_gamma_g[] = {0x3d3e3be1, 0x33323131, 0x0b070346, 0x110e100d,
	0x3e3b1710, 0x3231313d, 0x07034633, 0x0e100d0b, 0x00171011};
static u32 h8c7_gamma_b[] = {0x070601e2, 0x1f322a2d, 0x0e0c0540, 0x13121411,
	0x0601180f, 0x322a2d07, 0x0c05401f, 0x1214110e, 0x00180f13};

static u32 h8c7_mcs_protect_on[] = {0x000000b9};
static u32 h8c7_set_address_mode[] = {0x00000036};
static u32 h8c7_set_pixel_format[] = {0x0000703a};

static void mdfld_h8c7_dbi_ic_init(struct mdfld_dsi_config *dsi_config, int pipe)
{
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);
	unsigned long wait_timeout;

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return;
	}

	PSB_DEBUG_ENTRY("\n");

	/*wait for 5ms*/
	wait_timeout = jiffies + (HZ / 200);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/* sleep out and wait for 150ms. */
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_exit_sleep_mode, 1, 0);
	wait_timeout = jiffies + (3 * HZ / 20);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/* set password and wait for 10ms. */
	mdfld_dsi_send_gen_long_hs(sender, h8c7_mcs_protect_off, 1, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/* set TE on and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_tear_on, 1, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/* set backlight to full brightness and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_full_brightness, 1, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/* set backlight on and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_turn_on_backlight, 1, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/* disalble CABC and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_disable_cabc, 1, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_hs(sender, h8c7_ic_bias_current, 2, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_power, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_disp_reg, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_command_cyc, 6, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_mipi_ctrl, 1, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_hs(sender, h8c7_command_mode, 1, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_blanking_opt_2, 1, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_panel, 1, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_eq_func_ltps, 1, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_ltps_ctrl_output, 6, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_video_cyc, 6, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_hs(sender, h8c7_gamma_r, 9, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_hs(sender, h8c7_gamma_g, 9, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_gen_long_hs(sender, h8c7_gamma_b, 9, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/* disable password and wait for 10ms. */
	mdfld_dsi_send_gen_long_hs(sender, h8c7_mcs_protect_on, 1, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_address_mode, 1, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_pixel_format, 1, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
}

/* H8C7 DBI encoder helper funcs*/
static const struct drm_encoder_helper_funcs mdfld_h8c7_dbi_helper_funcs = {
	.dpms = mdfld_dsi_dbi_dpms,
	.mode_fixup = mdfld_dsi_mode_fixup,
	.prepare = mdfld_dsi_dbi_prepare,
	.mode_set = mdfld_dsi_dbi_mode_set,
	.commit = mdfld_dsi_dbi_commit,
};

/*H8C7 DBI encoder funcs*/
static const struct drm_encoder_funcs mdfld_h8c7_dbi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

void h8c7_cmd_init(struct drm_device* dev, struct panel_funcs* p_funcs)
{
	if (!dev || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	PSB_DEBUG_ENTRY("\n");

	p_funcs->encoder_funcs = &mdfld_h8c7_dbi_encoder_funcs;
	p_funcs->encoder_helper_funcs = &mdfld_h8c7_dbi_helper_funcs;
	p_funcs->get_config_mode = &h8c7_get_config_mode;
	p_funcs->update_fb = mdfld_dsi_dbi_update_fb;
	p_funcs->get_panel_info = h8c7_get_panel_info;
	p_funcs->reset = mdfld_dsi_h8c7_panel_reset;
	p_funcs->drv_ic_init = mdfld_h8c7_dbi_ic_init;
	p_funcs->detect = mdfld_dsi_h8c7_detect;
//	p_funcs->set_brightness = mdfld_dsi_tpo_cmd_set_brightness;
	p_funcs->set_brightness = mdfld_dsi_h8c7_set_brightness;
}
