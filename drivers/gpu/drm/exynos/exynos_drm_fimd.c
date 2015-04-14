/* exynos_drm_fimd.c
 *
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Authors:
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *	Inki Dae <inki.dae@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include "drmP.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/dispfreq.h>
#include <linux/irq.h>

#include <video/exynos_mipi_dsim.h>
#include <drm/exynos_drm.h>
#include <drm/drm_backlight.h>
#include <plat/regs-fb-v4.h>
#include <mach/map.h>

#include "exynos_drm_drv.h"
#include "exynos_drm_fbdev.h"
#include "exynos_drm_crtc.h"
#include "exynos_drm_iommu.h"
#ifdef CONFIG_DRM_EXYNOS_IPP
#include "exynos_drm_ipp.h"
#endif
#ifdef CONFIG_DRM_EXYNOS_DBG
#include "exynos_drm_dbg.h"
#endif

#include <plat/smies.h>

extern unsigned int lpcharge;

#define FIMD_HIGH_FRAMERATE 60
#define FIMD_LOW_FRAMERATE 30

#define RUNTIME_OFF_PERIOD	600 /* ms */
#define WIDTH_LIMIT			16

/*
 * FIMD is stand for Fully Interactive Mobile Display and
 * as a display controller, it transfers contents drawn on memory
 * to a LCD Panel through Display Interfaces such as RGB or
 * CPU Interface.
 */

/* position control register for hardware window 0, 2 ~ 4.*/
#define VIDOSD_A(win)		(VIDOSD_BASE + 0x00 + (win) * 16)
#define VIDOSD_B(win)		(VIDOSD_BASE + 0x04 + (win) * 16)
/* size control register for hardware window 0. */
#define VIDOSD_C_SIZE_W0	(VIDOSD_BASE + 0x08)
/* alpha control register for hardware window 1 ~ 4. */
#define VIDOSD_C(win)		(VIDOSD_BASE + 0x18 + (win) * 16)
/* size control register for hardware window 1 ~ 4. */
#define VIDOSD_D(win)		(VIDOSD_BASE + 0x0C + (win) * 16)

#define VIDWx_BUF_START(win, buf)	(VIDW_BUF_START(buf) + (win) * 8)
#define VIDWx_BUF_END(win, buf)		(VIDW_BUF_END(buf) + (win) * 8)
#define VIDWx_BUF_SIZE(win, buf)	(VIDW_BUF_SIZE(buf) + (win) * 4)

#define VIDW0nADD2_DWORD_CHECK(pagewidth, offsize)	\
		((pagewidth + offsize) % 8)

/* color key control register for hardware window 1 ~ 4. */
#define WKEYCON0_BASE(x)		((WKEYCON0 + 0x140) + (x * 8))
/* color key value register for hardware window 1 ~ 4. */
#define WKEYCON1_BASE(x)		((WKEYCON1 + 0x140) + (x * 8))

/* i80 interface control register */
#define I80IFCONFAx(x)			(0x1B0 + x * 4)
#define I80IFCONFBx(x)			(0x1B8 + x * 4)
#define LCD_CS_SETUP(x)			(x << 16)
#define LCD_WR_SETUP(x)			(x << 12)
#define LCD_WR_ACT(x)			(x << 8)
#define LCD_WR_HOLD(x)			(x << 4)
#define LCD_WR_RS_POL(x)		(x << 2)
#define I80IFEN_ENABLE			(1 << 0)

/* FIMD has totally five hardware windows. */
#define WINDOWS_NR	5
#ifdef CONFIG_DRM_EXYNOS_DBG
#define FIMD_MAX_REG	128
#endif

#define get_fimd_context(dev)	platform_get_drvdata(to_platform_device(dev))

/* memory type definitions. */
enum fimd_shadowcon {
	FIMD_SC_UNPROTECT	= 0 << 0,
	FIMD_SC_PROTECT	= 1 << 0,
	FIMD_SC_BYPASS_CH_CTRL	= 1 << 1,
	FIMD_SC_CH_DISABLE	= 0 << 2,
	FIMD_SC_CH_ENABLE	= 1 << 2,
};

struct fimd_driver_data {
	unsigned int timing_base;
	unsigned int lcdblk_off;
	unsigned int lcdblk_shift;

	unsigned int has_shadowcon:1;
	unsigned int has_clksel:1;
	unsigned int has_limited_fmt:1;
};

static struct fimd_driver_data s3c64xx_fimd_driver_data = {
	.timing_base = 0x0,
	.has_clksel = 1,
	.has_limited_fmt = 1,
};

static struct fimd_driver_data exynos3_fimd_driver_data = {
	.timing_base = 0x20000,
	.lcdblk_off = 0x210,
	.lcdblk_shift = 10,
	.has_shadowcon = 1,
};

static struct fimd_driver_data exynos4_fimd_driver_data = {
	.timing_base = 0x0,
	.lcdblk_off = 0x210,
	.lcdblk_shift = 10,
	.has_shadowcon = 1,
};

static struct fimd_driver_data exynos5_fimd_driver_data = {
	.timing_base = 0x20000,
	.lcdblk_off = 0x210,
	.lcdblk_shift = 24,
	.has_shadowcon = 1,
};

struct fimd_win_data {
	unsigned int		offset_x;
	unsigned int		offset_y;
	unsigned int		ovl_width;
	unsigned int		ovl_height;
	unsigned int		fb_width;
	unsigned int		fb_height;
	unsigned int		bpp;
	unsigned int		refresh;
	dma_addr_t		dma_addr;
	unsigned int		buf_offsize;
	unsigned int		line_size;	/* bytes */
	unsigned int		local_path;
	bool			enabled;
};

struct fimd_context {
	struct exynos_drm_subdrv	subdrv;
	struct device			*dev;
	struct drm_device		*drm_dev;
	struct device			*smies_device;
	int				irq;
	struct drm_crtc			*crtc;
	struct clk			*bus_clk;
	struct clk			*lcd_clk;
	struct resource			*regs_res;
	void __iomem			*regs;
	struct fimd_win_data		win_data[WINDOWS_NR];
	unsigned int			clkdiv;
	unsigned int			default_win;
	unsigned int			te_gpio;
	u32				vidcon0;
	u32				vidcon1;
	u32				i80ifcon;
	bool				i80_if;
	bool				iommu_on;
	int				pipe;
	struct mutex			lock;
	wait_queue_head_t		wait_vsync_queue;
	wait_queue_head_t		wait_te_queue;
	atomic_t			wait_vsync_event;
	atomic_t			wait_te_event;
	u32				no_trigger;
	atomic_t			win_updated;
	atomic_t			triggering;
	atomic_t			te_skip;
	int				dpms;
	atomic_t			smies_active;

	struct timer_list pm_timer;
	struct exynos_drm_panel_info *panel;
	struct notifier_block	nb_ctrl;
	struct fimd_driver_data *driver_data;
	struct platform_device		*disp_bus_pdev;
	struct dispfreq_device *dfd;
	struct workqueue_struct	*pm_wq;
	struct work_struct	pm_work;
	atomic_t			te_irq_on;
	atomic_t			do_apply;
	atomic_t			partial_requested;
	spinlock_t			win_updated_lock;
	bool				pm_gating_on;
	int	(*smies_on)(struct device *smies);
	int	(*smies_off)(struct device *smies);
	int	(*smies_mode)(struct device *smies, int mode);
	int				dbg_cnt;
};

static bool fimd_runtime_suspended(struct device *dev);
static int fimd_power_on(struct fimd_context *ctx, bool enable);
static void fimd_wait_for_vblank(struct device *dev);

static inline struct fimd_driver_data *drm_fimd_get_driver_data(
	struct platform_device *pdev)
{
	return (struct fimd_driver_data *)
		platform_get_device_id(pdev)->driver_data;
}

static int fimd_get_dpms(struct fimd_context *ctx, bool lock)
{
	int ret;

	if (!lock)
		return ctx->dpms;

	mutex_lock(&ctx->lock);
	ret = ctx->dpms;
	mutex_unlock(&ctx->lock);

	return ret;
}

static void fimd_pm_timer_handler(unsigned long arg)
{
	struct fimd_context *ctx = (struct fimd_context *)arg;
	struct drm_device *drm_dev = ctx->drm_dev;
	int crtc = ctx->pipe, dpms = fimd_get_dpms(ctx, 0);
	bool vbl_en = drm_dev->vblank_enabled[crtc];

	DRM_INFO("timer:wait_te[%d]vbl[%d]evt[%d %d %d]tr[%d]\n",
		atomic_read(&ctx->wait_te_event), vbl_en,
		atomic_read(&drm_dev->vblank_refcount[crtc]),
		exynos_drm_get_pendingflip(drm_dev, crtc),
		atomic_read(&ctx->win_updated),
		atomic_read(&ctx->triggering));

	if (!ctx->pm_gating_on) {
		DRM_INFO("%s:pm_gating is disabled\n", __func__);
		return;
	}

	if (dpms == DRM_MODE_DPMS_ON) {
		if (exynos_drm_get_pendingflip(drm_dev, crtc) ||
			atomic_read(&ctx->win_updated) ||
			atomic_read(&ctx->triggering)) {
			DRM_ERROR("remained event\n");
			ctx->dbg_cnt = 30;
		}

		if (!queue_work(ctx->pm_wq, &ctx->pm_work))
			DRM_INFO("%s:busy to queue_work.\n", __func__);
	} else
		DRM_INFO("%s:bypass for gate_off:cur_dpms[%d]\n",
				__func__, dpms);
}

static void fimd_activate_work(struct work_struct *work)
{
	struct fimd_context *ctx =
		container_of(work, struct fimd_context, pm_work);
	struct drm_device *drm_dev = ctx->drm_dev;
	struct mipi_dsim_device *dsim =
		platform_get_drvdata(ctx->disp_bus_pdev);
	struct device *smies_device = ctx->smies_device;
	int ret = 0, crtc = ctx->pipe;

	mutex_lock(&ctx->lock);

	DRM_INFO("gate_off:cur_dpms[%d]pm[%d %d]evt[%d %d %d]tr[%d]\n",
		fimd_get_dpms(ctx, 0),
		!fimd_runtime_suspended(ctx->dev),
		atomic_read(&ctx->dev->power.usage_count),
		atomic_read(&drm_dev->vblank_refcount[crtc]),
		exynos_drm_get_pendingflip(drm_dev, crtc),
		atomic_read(&ctx->win_updated),
		atomic_read(&ctx->triggering));

	if (fimd_get_dpms(ctx, 0) != DRM_MODE_DPMS_ON) {
		DRM_INFO("gate_off:bypass\n");
		goto bypass;
	}

	if (atomic_read(&ctx->triggering)){
		DRM_INFO("gate_off:wait vbl\n");

		fimd_wait_for_vblank(ctx->dev);

		if (atomic_read(&ctx->triggering)) {
			DRM_ERROR("handle triggering\n");
			atomic_set(&ctx->triggering, 0);
			mod_timer(&ctx->pm_timer, jiffies +
				((RUNTIME_OFF_PERIOD * DRM_HZ)/1000));
			goto bypass;
		}
	}

	exynos_drm_wait_finish_pageflip(drm_dev, crtc);

	if (atomic_read(&ctx->win_updated)) {
		DRM_ERROR("handle remained update\n");
		atomic_set(&ctx->win_updated, 0);
		exynos_drm_crtc_finish_pageflip(drm_dev, crtc);
	}

	if (atomic_read(&drm_dev->vblank_refcount[crtc])) {
		DRM_ERROR("handle remained vbl\n");
		drm_handle_vblank(drm_dev, crtc);
	}

	if (dsim && dsim->master_ops->runtime_active)
		ret = dsim->master_ops->runtime_active(dsim, false);

	if (atomic_read(&ctx->smies_active) && ctx->smies_off)
		ctx->smies_off(smies_device);

	ret = fimd_power_on(ctx, false);
	if (ret) {
		DRM_ERROR("fimd_power_on:ret[%d]\n", ret);
		goto recover;
	}

	ret = pm_runtime_put_sync(ctx->dev);
	if (ret) {
		DRM_ERROR("suspend:ret[%d]\n", ret);
		if (ret < 0)
			goto recover;
	}

	ctx->dpms = DRM_MODE_DPMS_SUSPEND;

	DRM_INFO("gate_off:done:cur_dpms[%d]pm[%d %d]evt[%d %d %d]tr[%d]\n",
		fimd_get_dpms(ctx, 0),
		!fimd_runtime_suspended(ctx->dev),
		atomic_read(&ctx->dev->power.usage_count),
		atomic_read(&drm_dev->vblank_refcount[crtc]),
		exynos_drm_get_pendingflip(drm_dev, crtc),
		atomic_read(&ctx->win_updated),
		atomic_read(&ctx->triggering));

	mutex_unlock(&ctx->lock);

	return;

recover:
	if (dsim && dsim->master_ops->runtime_active) {
		ret = dsim->master_ops->runtime_active(dsim, true);
		DRM_INFO("%s:mipi gate recover:ret[%d]",
			__func__, ret);
	}

bypass:
	DRM_INFO("gate_off:bypass:cur_dpms[%d]pm[%d %d]evt[%d %d %d]tr[%d]\n",
		fimd_get_dpms(ctx, 0),
		!fimd_runtime_suspended(ctx->dev),
		atomic_read(&ctx->dev->power.usage_count),
		atomic_read(&drm_dev->vblank_refcount[crtc]),
		exynos_drm_get_pendingflip(drm_dev, crtc),
		atomic_read(&ctx->win_updated),
		atomic_read(&ctx->triggering));

	mutex_unlock(&ctx->lock);

	return;
}

static int fimd_activate(struct fimd_context *ctx)
{
	struct mipi_dsim_device *dsim =
		platform_get_drvdata(ctx->disp_bus_pdev);
	struct device *smies_device = ctx->smies_device;
	int ret;

	mutex_lock(&ctx->lock);

	DRM_INFO("gate_on:cur_dpms[%d]pm[%d %d]\n",
		fimd_get_dpms(ctx, 0),
		!fimd_runtime_suspended(ctx->dev),
		atomic_read(&ctx->dev->power.usage_count));

	if (fimd_get_dpms(ctx, 0) != DRM_MODE_DPMS_SUSPEND) {
		DRM_INFO("gate_on:bypass\n");
		goto out;
	}

	ret = pm_runtime_get_sync(ctx->dev);
	if (ret)
		DRM_ERROR("resume:ret[%d]\n", ret);

	fimd_power_on(ctx, true);

	if (atomic_read(&ctx->smies_active) && ctx->smies_on)
		ctx->smies_on(smies_device);

	if (dsim && dsim->master_ops->runtime_active)
		dsim->master_ops->runtime_active(dsim, true);

out:
	DRM_INFO("gate_on:done:cur_dpms[%d]pm[%d %d]\n",
		fimd_get_dpms(ctx, 0),
		!fimd_runtime_suspended(ctx->dev),
		atomic_read(&ctx->dev->power.usage_count));

	mutex_unlock(&ctx->lock);

	return 0;
}

static void fimd_power_gate_set(struct fimd_context *ctx, bool enable)
{
	int dpms = fimd_get_dpms(ctx, 0);

	DRM_INFO("%s:cur_dpms[%d]cur_mode[%d]\n",
			__func__, dpms, ctx->pm_gating_on);

	if (ctx->pm_gating_on == enable) {
		DRM_ERROR("invalid mode set[%d]\n", enable);
		return;
	}

	if (enable) {
		if (dpms != DRM_MODE_DPMS_ON) {
			DRM_ERROR("invalid dpms[%d]\n", dpms);
			return;
		}

		setup_timer(&ctx->pm_timer, fimd_pm_timer_handler,
			    (unsigned long)ctx);
	} else {
		del_timer(&ctx->pm_timer);

		if (dpms == DRM_MODE_DPMS_SUSPEND)
			fimd_activate(ctx);
	}

	ctx->pm_gating_on = enable;

	DRM_INFO("%s:done:cur_dpms[%d]cur_mode[%d]\n",
			__func__, dpms, ctx->pm_gating_on);
}

static inline void fimd_prepare_i80_access(struct fimd_context *ctx, const char *str)
{
	if (!ctx->pm_gating_on)
		return;

	/* enable fimd clock if it was disabled by runtime activate. */
	if (fimd_get_dpms(ctx, 1) == DRM_MODE_DPMS_SUSPEND) {
		if (str)
			DRM_INFO("gate_on[%s]\n", str);

		fimd_activate(ctx);
	}

	mod_timer(&ctx->pm_timer,
		  jiffies + ((RUNTIME_OFF_PERIOD * DRM_HZ)/1000));
}

static int fimd_set_runtime_activate(struct device *dev)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	struct drm_device *drm_dev = ctx->drm_dev;
	bool vbl_en = drm_dev->vblank_enabled[ctx->pipe];

	if (!ctx->i80_if)
		return EPERM;

	if (!ctx->pm_gating_on)
		return 0;

	DRM_INFO("active:cur_dpms[%d]vbl[%d]tr[%d]\n",
		fimd_get_dpms(ctx, 0), vbl_en,
		atomic_read(&ctx->triggering));

	fimd_prepare_i80_access(ctx, __func__);

	return 0;
}

#ifdef CONFIG_EXYNOS_SMIES
#ifdef CONFIG_EXYNOS_SMIES_DEFALUT_ENABLE
static int fimd_set_smies_mode(struct device *dev, int mode)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	struct device *smies_device = ctx->smies_device;
	int dpms = fimd_get_dpms(ctx, 0);

	if (!ctx->i80_if)
		return EPERM;

	if (dpms ==DRM_MODE_DPMS_ON)
		ctx->smies_mode(smies_device, mode);

	return 0;
}

#else
static int fimd_set_smies_activate(struct device *dev, bool enable)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	struct device *smies_device = ctx->smies_device;
	int dpms = fimd_get_dpms(ctx, 0);

	if (!ctx->i80_if)
		return EPERM;

	atomic_set(&ctx->smies_active, enable);

	if (dpms ==DRM_MODE_DPMS_ON) {
		if (atomic_read(&ctx->smies_active))
			ctx->smies_on(smies_device);
		else
			ctx->smies_off(smies_device);
	}

	return 0;
}
#endif
#endif

static void fimd_update_panel_refresh(struct device *dev, unsigned int rate)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	struct exynos_drm_panel_info *panel = ctx->panel;

	/* panel notify the self refresh rate to fimd
	* so that fimd decide whether it skip te signal or not.
	*/
	panel->self_refresh = rate;
}

static void fimd_trigger(struct device *dev)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	struct drm_device *drm_dev = ctx->drm_dev;
	struct mipi_dsim_device *dsim;
	u32 reg;
	bool vbl_en = drm_dev->vblank_enabled[ctx->pipe];
	static bool first_open = true;

	if (first_open) {
		if (!ctx->drm_dev || !ctx->drm_dev->open_count)
			return;

		first_open = false;
	}

	/*
	 * no_trigger == 1 means that MIPI-DSI tries to transfer some commands
	 * to lcd panel. So do not trigger.
	 */
	if (ctx->no_trigger)
		return;

	/*
	 * FIMD has to guarantee not to power off until trigger action is
	 * completed, i.e, has to skip i80 time out event handler during
	 * triggering mode. So needs triggering flag to check it.
	 * And the triggering flag is set just before setting i80 frame done
	 * interrupt and unset just after unsetting i80 frame done interrupt.
	 */

	/* enter triggering mode */
	atomic_set(&ctx->triggering, 1);

	if (ctx->dbg_cnt) {
		DRM_INFO("TRIG[0x%x 0x%x]c[%d]\n",
			readl(ctx->regs + VIDINTCON0),
			readl(ctx->regs + TRIGCON),
			ctx->dbg_cnt);
	}

	/* set i80 fame done interrupt */
	reg = readl(ctx->regs + VIDINTCON0);
	reg |= (VIDINTCON0_INT_ENABLE | VIDINTCON0_INT_I80IFDONE |
						VIDINTCON0_INT_SYSMAINCON);
	writel(reg, ctx->regs + VIDINTCON0);

	dsim = platform_get_drvdata(ctx->disp_bus_pdev);
	if (dsim && dsim->master_ops->set_clock_mode)
		dsim->master_ops->set_clock_mode(dsim, 1);

	reg = readl(ctx->regs + TRIGCON);

	if (!vbl_en)
		DRM_INFO("%s:vbl_off[0x%x]\n", __func__, reg);

	reg |= 1 << 0 | 1 << 1;
	writel(reg, ctx->regs + TRIGCON);
}

static bool fimd_display_is_connected(struct device *dev)
{
	DRM_DEBUG_KMS("%s\n", __FILE__);

	/* TODO. */

	return true;
}

static void *fimd_get_panel(struct device *dev)
{
	struct fimd_context *ctx = get_fimd_context(dev);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	return ctx->panel;
}

static int fimd_check_timing(struct device *dev, void *timing)
{
	DRM_DEBUG_KMS("%s\n", __FILE__);

	/* TODO. */

	return 0;
}

static int fimd_display_power_on(struct device *dev, int mode)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	struct mipi_dsim_device *dsim =
		platform_get_drvdata(ctx->disp_bus_pdev);
	struct device *smies_device = ctx->smies_device;

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		mutex_lock(&ctx->lock);
		DRM_INFO("fimd_bl_on:cur_dpms[%d]pm[%d %d]\n",
			fimd_get_dpms(ctx, 0),
			!fimd_runtime_suspended(ctx->dev),
			atomic_read(&ctx->dev->power.usage_count));

		drm_bl_dpms(mode);

		DRM_INFO("fimd_bl_on:done:cur_dpms[%d]pm[%d %d]\n",
			fimd_get_dpms(ctx, 0),
			!fimd_runtime_suspended(ctx->dev),
			atomic_read(&ctx->dev->power.usage_count));
		mutex_unlock(&ctx->lock);
		fimd_prepare_i80_access(ctx, "fimd_bl_on");
		break;
	case DRM_MODE_DPMS_OFF:
		mutex_lock(&ctx->lock);
		DRM_INFO("fimd_bl_off:cur_dpms[%d]pm[%d %d]\n",
			fimd_get_dpms(ctx, 0),
			!fimd_runtime_suspended(ctx->dev),
			atomic_read(&ctx->dev->power.usage_count));

		if (fimd_get_dpms(ctx, 0) == DRM_MODE_DPMS_SUSPEND) {
			pm_runtime_get_sync(ctx->dev);
			fimd_power_on(ctx, true);

			if (atomic_read(&ctx->smies_active) && ctx->smies_on)
				ctx->smies_on(smies_device);
			if (dsim && dsim->master_ops->runtime_active)
				dsim->master_ops->runtime_active(dsim, true);
		}

		drm_bl_dpms(mode);

		DRM_INFO("fimd_bl_off:done:cur_dpms[%d]pm[%d %d]\n",
			fimd_get_dpms(ctx, 0),
			!fimd_runtime_suspended(ctx->dev),
			atomic_read(&ctx->dev->power.usage_count));
		mutex_unlock(&ctx->lock);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	default:
		DRM_INFO("unspecified mode %d\n", mode);
		break;
	}

	return 0;
}

static void fimd_partial_resolution(struct device *dev,
					struct exynos_drm_partial_pos *pos)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	struct fimd_driver_data *driver_data = ctx->driver_data;
	void *timing_base = ctx->regs + driver_data->timing_base;
	u32 val;
	int dpms = fimd_get_dpms(ctx, 0);

	if (dpms > 0) {
		DRM_ERROR("invalid:cur_dpms[%d]\n", dpms);
		return;
	}

	DRM_DEBUG_KMS("%s:res[%d %d]\n", __func__, pos->w, pos->h);

	/* setup horizontal and vertical display size. */
	val = VIDTCON2_LINEVAL(pos->h - 1) | VIDTCON2_HOZVAL(pos->w - 1) |
	       VIDTCON2_LINEVAL_E(pos->h - 1) | VIDTCON2_HOZVAL_E(pos->w - 1);

	writel(val, timing_base + VIDTCON2);
}

static void fimd_request_partial_update(struct device *dev)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	unsigned long flags;

	DRM_DEBUG_KMS("%s\n", __func__);

	spin_lock_irqsave(&ctx->win_updated_lock, flags);
	atomic_set(&ctx->win_updated, 1);
	spin_unlock_irqrestore(&ctx->win_updated_lock, flags);
	atomic_set(&ctx->partial_requested, 1);
}

static void fimd_adjust_partial_region(struct device *dev,
					struct exynos_drm_partial_pos *pos)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	struct exynos_drm_panel_info *panel = ctx->panel;
	struct fb_videomode *timing = &panel->timing;
	unsigned int old_x;
	unsigned int old_w;

	old_x = pos->x;
	old_w = pos->w;

	/*
	 * if pos->x is bigger than 0, adjust pos->x and pos->w roughly.
	 *
	 * ######################################### <- image
	 *	|	|		|
	 *	|	before(x)	before(w)
	 *	after(x)		after(w)
	 */
	if (pos->x > 0) {
		pos->x = ALIGN(pos->x, WIDTH_LIMIT) - WIDTH_LIMIT;
		pos->w += WIDTH_LIMIT;
	}

	/*
	 * pos->w should be WIDTH_LIMIT if pos->w is 0, and also pos->x should
	 * be adjusted properly if pos->x is bigger than WIDTH_LIMIT.
	 */
	if (pos->w == 0) {
		if (pos->x > WIDTH_LIMIT)
			pos->x -= WIDTH_LIMIT;
		pos->w = WIDTH_LIMIT;
	} else
		pos->w = ALIGN(pos->w, WIDTH_LIMIT);

	/*
	 * pos->x + pos->w should be smaller than horizontal size of display.
	 * If not so, page fault exception will be occurred.
	 */
	if (pos->x + pos->w > timing->xres)
		pos->w = timing->xres - pos->x;

	DRM_DEBUG_KMS("%s:adjusted:x[%d->%d]w[%d->%d]\n",
			__func__, old_x, pos->x, old_w, pos->w);
}

static int fimd_set_partial_region(struct device *dev,
					struct exynos_drm_partial_pos *pos)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	struct mipi_dsim_device *dsim;

	DRM_DEBUG_KMS("%s:pos[%d %d %d %d]\n",
		__func__, pos->x, pos->y, pos->w, pos->h);

	dsim = platform_get_drvdata(ctx->disp_bus_pdev);
	if (dsim && dsim->master_ops->set_partial_region)
		dsim->master_ops->set_partial_region(dsim, pos);

	return 0;
}

static struct exynos_drm_display_ops fimd_display_ops = {
	.type = EXYNOS_DISPLAY_TYPE_LCD,
	.is_connected = fimd_display_is_connected,
	.get_panel = fimd_get_panel,
	.check_timing = fimd_check_timing,
	.power_on = fimd_display_power_on,
	.set_partial_region = fimd_set_partial_region,
};

static void fimd_dpms(struct device *subdrv_dev, int mode)
{
	struct fimd_context *ctx = get_fimd_context(subdrv_dev);
	struct device *smies_device = ctx->smies_device;
	int ret;

	mutex_lock(&ctx->lock);

	DRM_INFO("%s[%d]cur_dpms[%d]pm[%d %d]\n",
		__func__, mode, fimd_get_dpms(ctx, 0),
		!fimd_runtime_suspended(ctx->dev),
		atomic_read(&ctx->dev->power.usage_count));

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		/*
		 * enable fimd hardware only if suspended status.
		 *
		 * P.S. fimd_dpms function would be called at booting time so
		 * clk_enable could be called double time.
		 */
		if (fimd_get_dpms(ctx, 0) == DRM_MODE_DPMS_OFF) {
			ret = pm_runtime_get_sync(subdrv_dev);
			if (ret)
				DRM_ERROR("resume:ret[%d]\n", ret);

			fimd_power_on(ctx, true);

			if (atomic_read(&ctx->smies_active) && ctx->smies_on)
				ctx->smies_on(smies_device);
		}
		break;
	case DRM_MODE_DPMS_OFF:
		if (fimd_get_dpms(ctx, 0) == DRM_MODE_DPMS_ON) {
			if (atomic_read(&ctx->smies_active) && ctx->smies_off)
				ctx->smies_off(smies_device);

			ret = fimd_power_on(ctx, false);
			if (ret) {
				DRM_ERROR("fimd_power_on:ret[%d]\n", ret);
				goto out;
			}

			ret = pm_runtime_put_sync(subdrv_dev);
			if (ret) {
				DRM_ERROR("suspend:ret[%d]\n", ret);
				if (ret < 0)
					goto out;
			}
		}
		ctx->dpms = DRM_MODE_DPMS_OFF;
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	default:
		DRM_INFO("unspecified mode %d\n", mode);
		break;
	}

out:
	DRM_INFO("%s:done[%d]cur_dpms[%d]pm[%d %d]\n",
		__func__, mode, fimd_get_dpms(ctx, 0),
		!fimd_runtime_suspended(ctx->dev),
		atomic_read(&ctx->dev->power.usage_count));

	mutex_unlock(&ctx->lock);
}

static void fimd_apply(struct device *subdrv_dev)
{
	struct fimd_context *ctx = get_fimd_context(subdrv_dev);
	struct exynos_drm_manager *mgr = ctx->subdrv.manager;
	struct exynos_drm_manager_ops *mgr_ops = mgr->ops;
	struct exynos_drm_overlay_ops *ovl_ops = mgr->overlay_ops;
	struct fimd_win_data *win_data;
	int i;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	atomic_set(&ctx->do_apply, 1);

	for (i = 0; i < WINDOWS_NR; i++) {
		win_data = &ctx->win_data[i];
		if (win_data->enabled && (ovl_ops && ovl_ops->commit))
			ovl_ops->commit(subdrv_dev, i);
	}

	if (mgr_ops && mgr_ops->commit)
		mgr_ops->commit(subdrv_dev);

	atomic_set(&ctx->do_apply, 0);
}

static void fimd_sysreg_setup(struct fimd_context *ctx)
{
	unsigned int reg = 0;
	/*
	 * Set DISP1BLK_CFG register for Display path selection
	 * ---------------------
	 *  0 | MIE/MDNIE
	 *  1 | FIMD : selected
	 */
	reg = __raw_readl(S3C_VA_SYS + 0x0210);
	reg &= ~(1 << 1);
#if !defined(CONFIG_EXYNOS_SMIES_ENABLE_BOOTTIME)
	if (!atomic_read(&ctx->smies_active))
		reg |= (1 << 1);
#endif
	__raw_writel(reg, S3C_VA_SYS + 0x0210);

	reg = __raw_readl(S3C_VA_SYS + 0x0210);
	reg &= ~(3 << 10);
	reg |= (1 << 10);
	__raw_writel(reg, S3C_VA_SYS + 0x0210);
}

static void fimd_commit(struct device *dev)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	struct exynos_drm_panel_info *panel = ctx->panel;
	struct fb_videomode *timing = &panel->timing;
	struct fimd_driver_data *driver_data;
	u32 val;
	int dpms = fimd_get_dpms(ctx, 0);

	if (dpms > 0) {
		DRM_ERROR("invalid:cur_dpms[%d]\n", dpms);
		return;
	}

	driver_data = ctx->driver_data;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (ctx->i80_if) {
		/* If it's not mipi module, also set RSPOL bit to high. */
		val = ctx->i80ifcon | I80IFEN_ENABLE;

		writel(val, ctx->regs + driver_data->timing_base +
			I80IFCONFAx(0));

		/* Disable auto frame rate. */
		writel(0, ctx->regs + driver_data->timing_base +
			I80IFCONFBx(0));

		val = readl(ctx->regs + VIDOUT_CON);
		val &= ~VIDOUT_CON_F_MASK;
		val |= VIDOUT_CON_F_I80_LDI0;
		writel(val, ctx->regs + VIDOUT_CON);

		/* Set i80 interface bit to LCDBLK_CFG register. */
		fimd_sysreg_setup(ctx);
	} else {
		/* setup polarity values from machine code. */
		writel(ctx->vidcon1,
			ctx->regs + driver_data->timing_base + VIDCON1);

		/* setup vertical timing values. */
		val = VIDTCON0_VBPD(timing->upper_margin - 1) |
		       VIDTCON0_VFPD(timing->lower_margin - 1) |
		       VIDTCON0_VSPW(timing->vsync_len - 1);
		writel(val, ctx->regs + driver_data->timing_base + VIDTCON0);

		/* setup horizontal timing values.  */
		val = VIDTCON1_HBPD(timing->left_margin - 1) |
		       VIDTCON1_HFPD(timing->right_margin - 1) |
		       VIDTCON1_HSPW(timing->hsync_len - 1);
		writel(val, ctx->regs + driver_data->timing_base + VIDTCON1);
	}

	/* setup horizontal and vertical display size. */
	val = VIDTCON2_LINEVAL(timing->yres - 1) |
	       VIDTCON2_HOZVAL(timing->xres - 1);
	writel(val, ctx->regs + driver_data->timing_base + VIDTCON2);

#ifdef CONFIG_EXYNOS_SMIES
	val = readl(ctx->regs + 0x2001C);
	val |= VIDTCON3_VSYNCEN;
	writel(val, ctx->regs + 0x2001C);
#endif

	/* setup clock source, clock divider, enable dma. */
	val = ctx->vidcon0;
	val &= ~(VIDCON0_CLKVAL_F_MASK | VIDCON0_CLKDIR);

	if (ctx->clkdiv > 1)
		val |= VIDCON0_CLKVAL_F(ctx->clkdiv - 1) | VIDCON0_CLKDIR;
	else
		val &= ~VIDCON0_CLKDIR;	/* 1:1 clock */

#ifdef CONFIG_EXYNOS_SMIES
	val |= VIDCON0_HIVCLK | VIDCON0_VCLKFREE;
#endif
	/*
	 * fields of register with prefix '_F' would be updated
	 * at vsync(same as dma start)
	 */
	val |= VIDCON0_ENVID | VIDCON0_ENVID_F;
	writel(val, ctx->regs + VIDCON0);
}

static void fimd_prepare_vblank(struct device *dev)
{
	struct fimd_context *ctx = get_fimd_context(dev);

	if (ctx->i80_if)
		fimd_prepare_i80_access(ctx, "vbl");
}

static int fimd_enable_vblank(struct device *dev)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	u32 val;
	int dpms = fimd_get_dpms(ctx, 0);

	DRM_INFO("vbl_on:cur_dpms[%d]tr[%d]\n", dpms,
		atomic_read(&ctx->triggering));

	if (dpms > 0) {
		DRM_ERROR("invalid:cur_dpms[%d]\n", dpms);
		return -EPERM;
	}

	val = readl(ctx->regs + VIDINTCON0);

	val |= (VIDINTCON0_INT_ENABLE | VIDINTCON0_INT_FRAME);
	val &= ~VIDINTCON0_FRAMESEL0_MASK;
	val |= VIDINTCON0_FRAMESEL0_VSYNC;
	val &= ~VIDINTCON0_FRAMESEL1_MASK;
	val |= VIDINTCON0_FRAMESEL1_NONE;
	DRM_INFO("vbl_on[0x%x->0x%x]\n",
		readl(ctx->regs + VIDINTCON0), val);

	writel(val, ctx->regs + VIDINTCON0);

	return 0;
}

static void fimd_disable_vblank(struct device *dev)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	struct drm_device *drm_dev = ctx->drm_dev;
	u32 val;
	int dpms = fimd_get_dpms(ctx, 0);
	bool vbl_en = drm_dev->vblank_enabled[ctx->pipe];

	DRM_INFO("vbl_off:cur_dpms[%d]wait_te[%d]vbl[%d]tr[%d]\n",
		dpms, atomic_read(&ctx->wait_te_event), vbl_en,
		atomic_read(&ctx->triggering));

	if (dpms > 0) {
		DRM_INFO("vbl_off:bypass\n");
		return;
	}

	val = readl(ctx->regs + VIDINTCON0);

	val &= ~(VIDINTCON0_INT_ENABLE | VIDINTCON0_INT_FRAME);
	DRM_INFO("vbl_off[0x%x->0x%x]\n",
		readl(ctx->regs + VIDINTCON0), val);

	writel(val, ctx->regs + VIDINTCON0);
}

static void fimd_wait_for_vblank(struct device *dev)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	int dpms = fimd_get_dpms(ctx, 0);

	if (dpms > 0) {
		DRM_ERROR("invalid:cur_dpms[%d]\n", dpms);
		return;
	}

	atomic_set(&ctx->wait_vsync_event, 1);

	/*
	 * wait for FIMD to signal VSYNC interrupt or return after
	 * timeout which is set to 50ms (refresh rate of 20).
	 */
	if (!wait_event_timeout(ctx->wait_vsync_queue,
				!atomic_read(&ctx->wait_vsync_event),
				DRM_HZ/20))
		DRM_ERROR("vblank wait timed out.\n");
}

static void fimd_wait_for_te_signal(struct device *dev)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	struct drm_device *drm_dev = ctx->drm_dev;
	struct exynos_drm_panel_info *panel = ctx->panel;
	struct fb_videomode *timing = &panel->timing;
	bool vbl_en = drm_dev->vblank_enabled[ctx->pipe];
	int dpms = fimd_get_dpms(ctx, 0);
	u32 delay = timing->refresh < panel->self_refresh ?
		DRM_HZ/10 : DRM_HZ/20;

	if (dpms > 0) {
		DRM_INFO("%s:bypass:cur_dpms[%d]\n", __func__, dpms);
		return;
	}

	atomic_inc(&ctx->wait_te_event);

	DRM_INFO("wait_te[%d]vbl[%d]tr[%d]\n",
		atomic_read(&ctx->wait_te_event), vbl_en,
		atomic_read(&ctx->triggering));

	/*
	 * wait for TE signal interrupt or return after
	 * timeout which is set to 50ms (refresh rate of 20).
	 */
	if (!wait_event_timeout(ctx->wait_te_queue,
				!atomic_read(&ctx->wait_te_event),
				delay)) {
		DRM_ERROR("timeout:irq[%d]reg[0x%x]stop_tr[%d]trig[0x%x]\n",
				atomic_read(&ctx->te_irq_on),
				readl(ctx->regs + VIDINTCON0),
				ctx->no_trigger,
				readl(ctx->regs + TRIGCON));
		ctx->dbg_cnt = 30;
	}
}

static void fimd_wait_for_completed_signal(struct device *dev)
{
	struct fimd_context *ctx = get_fimd_context(dev);

	if (ctx->i80_if)
		fimd_wait_for_te_signal(dev);
	else
		fimd_wait_for_vblank(dev);
}

static void fimd_stop_trigger(struct device *dev,
					unsigned int stop)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	int dpms = fimd_get_dpms(ctx, 0);

	if (dpms > 0) {
		DRM_ERROR("invalid:cur_dpms[%d]\n", dpms);
		return;
	}

	DRM_INFO("stop_tr[%d->%d]\n", ctx->no_trigger, stop);

	ctx->no_trigger = stop;
}

irqreturn_t panel_te_interrupt(int irq, void *dev_id)
{
	struct fimd_context *ctx = (struct fimd_context *)dev_id;
	struct drm_device *drm_dev = ctx->drm_dev;
	struct exynos_drm_panel_info *panel = ctx->panel;
	struct fb_videomode *timing = &panel->timing;
	int crtc = ctx->pipe;
	unsigned long flags;

	DRM_DEBUG_KMS("%d\n", irq);

	/* check the crtc is detached already from encoder */
	if (crtc < 0 || !drm_dev) {
		DRM_ERROR("crtc is detached\n");
		return IRQ_HANDLED;
	}

	/*
	 * The ctx->win_updated has to be protected not to change until
	 * fimd_trigger() is called.
	 */
	spin_lock_irqsave(&ctx->win_updated_lock, flags);

	if (ctx->dbg_cnt)
		DRM_INFO("TE:%s%s%s%s:tr[%d]evt[%d %d]c[%d]\n",
			atomic_read(&ctx->te_skip) ? "[skip]" : "",
			atomic_read(&ctx->wait_te_event) ? "[wait_te]" : "",
			atomic_read(&ctx->win_updated) ? "[ui]" : "",
			ctx->no_trigger ? "[no_tr]" : "",
			atomic_read(&ctx->triggering),
			atomic_read(&drm_dev->vblank_refcount[crtc]),
			exynos_drm_get_pendingflip(drm_dev, crtc),
			ctx->dbg_cnt--);

	/*
	 * If refresh is low, pending condition check.
	 *
	 * This supports low framerate without panel flicker.
	 */
	if (atomic_read(&ctx->te_skip) &&
	    timing->refresh < panel->self_refresh) {
		atomic_set(&ctx->te_skip, 0);
		spin_unlock_irqrestore(&ctx->win_updated_lock, flags);
		return IRQ_HANDLED;
	}

	/* set wait te event to zero and wake up queue. */
	if (atomic_read(&ctx->wait_te_event)) {
		atomic_set(&ctx->wait_te_event, 0);
		DRM_WAKEUP(&ctx->wait_te_queue);

		/*
		 * If ctx->win_updated = 1 then set it to 0 so that
		 * fimd_trigger isn't called.
		 * This resolves page fault issus that it can be occurred
		 * when rmfb is requested between win_commit and te interrupt
		 * handler.
		 */
		if (atomic_read(&ctx->win_updated)) {
			atomic_set(&ctx->win_updated, 0);
			spin_unlock_irqrestore(&ctx->win_updated_lock, flags);
			exynos_drm_crtc_finish_pageflip(drm_dev, crtc);
			goto out_handle_vblank;
		}
	}

	/*
	 * If trigger isn't required, just return.
	 *
	 * This is for continuing to queue updating request from user
	 * even if the trigger isn't required.
	 */
	if (ctx->no_trigger) {
		spin_unlock_irqrestore(&ctx->win_updated_lock, flags);
		return IRQ_HANDLED;
	}

	/*
	 * If there is previous pageflip request, do trigger and handle the
	 * pageflip event so that current framebuffer can be updated into GRAM
	 * of lcd panel.
	 */
	if (atomic_read(&ctx->win_updated)) {
		spin_unlock_irqrestore(&ctx->win_updated_lock, flags);
		/*
		 * skip in triggering mode because multiple triggering can
		 * cause panel reset
		 */
		if (!atomic_read(&ctx->triggering)) {
			if (atomic_read(&ctx->partial_requested)) {
				request_crtc_partial_update(drm_dev, crtc);
				atomic_set(&ctx->partial_requested, 0);
			}

			spin_lock_irqsave(&ctx->win_updated_lock,
						flags);
			atomic_set(&ctx->win_updated, 0);
			spin_unlock_irqrestore(&ctx->win_updated_lock,
						flags);

			fimd_trigger(ctx->dev);
			exynos_drm_crtc_finish_pageflip(drm_dev, crtc);
		}

		spin_lock_irqsave(&ctx->win_updated_lock, flags);
	}

	spin_unlock_irqrestore(&ctx->win_updated_lock, flags);

out_handle_vblank:
	/*
	 * Return vblank event to user process only if vblank interrupt
	 * is enabled.
	 */
	if (atomic_read(&drm_dev->vblank_refcount[crtc]))
		drm_handle_vblank(drm_dev, crtc);

	atomic_set(&ctx->te_skip, 1);

	return IRQ_HANDLED;
}

static int fimd_subdrv_probe(struct drm_device *drm_dev, struct device *dev)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	int pipe = 0;

	ctx->drm_dev = drm_dev;
	ctx->pipe = pipe;

	/*
	 * enable drm irq mode.
	 * - with irq_enabled = 1, we can use the vblank feature.
	 *
	 * P.S. note that we wouldn't use drm irq handler but
	 *	just specific driver own one instead because
	 *	drm framework supports only one irq handler.
	 */
	drm_dev->irq_enabled = 1;

	/*
	 * with vblank_disable_allowed = 1, vblank interrupt will be disabled
	 * by drm timer once a current process gives up ownership of
	 * vblank event.(after drm_vblank_put function is called)
	 */
	drm_dev->vblank_disable_allowed = 1;

	return 0;
}

static void fimd_subdrv_remove(struct drm_device *drm_dev, struct device *dev)
{
	DRM_DEBUG_KMS("%s\n", __FILE__);

	/* detach this sub driver from iommu mapping if supported. */
	if (is_drm_iommu_supported(drm_dev))
		drm_iommu_detach_device(drm_dev, dev);

	/* TODO. */
}

static struct exynos_drm_manager_ops fimd_manager_ops = {
	.dpms = fimd_dpms,
	.apply = fimd_apply,
	.commit = fimd_commit,
	.prepare_vblank = fimd_prepare_vblank,
	.enable_vblank = fimd_enable_vblank,
	.disable_vblank = fimd_disable_vblank,
	.wait_for_vblank = fimd_wait_for_completed_signal,
};

static void fimd_win_mode_set(struct device *dev,
			      struct exynos_drm_overlay *overlay)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	struct exynos_drm_panel_info *panel = ctx->panel;
	struct fb_videomode *timing = &panel->timing;
	struct fimd_win_data *win_data;
	struct mipi_dsim_device *dsim;
	int win;
	unsigned long offset;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (!overlay) {
		dev_err(dev, "overlay is NULL\n");
		return;
	}

	win = overlay->zpos;
	if (win == DEFAULT_ZPOS)
		win = ctx->default_win;

	if (win < 0 || win > WINDOWS_NR)
		return;

	offset = overlay->fb_x * (overlay->bpp >> 3);
	offset += overlay->fb_y * overlay->pitch;

	DRM_DEBUG_KMS("offset = 0x%lx, pitch = %x\n", offset, overlay->pitch);

	win_data = &ctx->win_data[win];

	win_data->offset_x = overlay->crtc_x;
	win_data->offset_y = overlay->crtc_y;
	win_data->ovl_width = overlay->crtc_width;
	win_data->ovl_height = overlay->crtc_height;
	win_data->fb_width = overlay->fb_width;
	win_data->fb_height = overlay->fb_height;
	win_data->dma_addr = overlay->dma_addr[0] + offset;
	win_data->bpp = overlay->bpp;
	win_data->buf_offsize = (overlay->fb_width - overlay->crtc_width) *
				(overlay->bpp >> 3);
	win_data->line_size = overlay->crtc_width * (overlay->bpp >> 3);
	win_data->refresh = overlay->refresh;

	if (timing->refresh != win_data->refresh) {
		/* FIXME: fimd clock configure ? */

		/* MIPI configuration */
		dsim = platform_get_drvdata(ctx->disp_bus_pdev);
		if (dsim && dsim->master_ops->set_refresh_rate)
			dsim->master_ops->set_refresh_rate(dsim,
				win_data->refresh);

		timing->refresh = win_data->refresh;
	}

	DRM_DEBUG_KMS("offset_x = %d, offset_y = %d\n",
			win_data->offset_x, win_data->offset_y);
	DRM_DEBUG_KMS("ovl_width = %d, ovl_height = %d\n",
			win_data->ovl_width, win_data->ovl_height);
	DRM_DEBUG_KMS("paddr = 0x%lx\n",
			(unsigned long)win_data->dma_addr);
	DRM_DEBUG_KMS("fb_width = %d, crtc_width = %d\n",
			overlay->fb_width, overlay->crtc_width);
	DRM_DEBUG_KMS("refresh = %d\n", win_data->refresh);
}

static void fimd_win_set_pixfmt(struct device *dev, unsigned int win)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	struct fimd_win_data *win_data = &ctx->win_data[win];
	unsigned long val;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	val = WINCONx_ENWIN;

	switch (win_data->bpp) {
	case 1:
		val |= WINCON0_BPPMODE_1BPP;
		val |= WINCONx_BITSWP;
		val |= WINCONx_BURSTLEN_4WORD;
		break;
	case 2:
		val |= WINCON0_BPPMODE_2BPP;
		val |= WINCONx_BITSWP;
		val |= WINCONx_BURSTLEN_8WORD;
		break;
	case 4:
		val |= WINCON0_BPPMODE_4BPP;
		val |= WINCONx_BITSWP;
		val |= WINCONx_BURSTLEN_8WORD;
		break;
	case 8:
		val |= WINCON0_BPPMODE_8BPP_PALETTE;
		val |= WINCONx_BURSTLEN_8WORD;
		val |= WINCONx_BYTSWP;
		break;
	case 16:
		val |= WINCON0_BPPMODE_16BPP_565;
		val |= WINCONx_HAWSWP;
		val |= WINCONx_BURSTLEN_16WORD;
		break;
	case 24:
		val |= WINCON0_BPPMODE_24BPP_888;
		val |= WINCONx_WSWP;
		val |= WINCONx_BURSTLEN_16WORD;
		break;
	case 32:
		val |= WINCON1_BPPMODE_28BPP_A4888
			| WINCON1_BLD_PIX | WINCON1_ALPHA_SEL;
		val |= WINCONx_WSWP;
		val |= WINCONx_BURSTLEN_16WORD;
		break;
	default:
		DRM_DEBUG_KMS("invalid pixel size so using unpacked 24bpp.\n");

		val |= WINCON0_BPPMODE_24BPP_888;
		val |= WINCONx_WSWP;
		val |= WINCONx_BURSTLEN_16WORD;
		break;
	}

	DRM_DEBUG_KMS("bpp = %d\n", win_data->bpp);

	writel(val, ctx->regs + WINCON(win));
}

static void fimd_win_set_colkey(struct device *dev, unsigned int win)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	unsigned int keycon0 = 0, keycon1 = 0;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	keycon0 = ~(WxKEYCON0_KEYBL_EN | WxKEYCON0_KEYEN_F |
			WxKEYCON0_DIRCON) | WxKEYCON0_COMPKEY(0);

	keycon1 = WxKEYCON1_COLVAL(0xffffffff);

	writel(keycon0, ctx->regs + WKEYCON0_BASE(win));
	writel(keycon1, ctx->regs + WKEYCON1_BASE(win));
}

static int fimd_check_min_width(struct fimd_win_data *win_data, u32 *min_width)
{
	int burst_length, bpp;

	bpp = (win_data->bpp >> 3);

	switch (win_data->bpp) {
	case 1:
		burst_length = 4;
		break;
	case 2 ... 8:
		burst_length = 8;
		break;
	case 16 ... 32:
		burst_length = 16;
		break;
	default:
		return -EINVAL;
	}

	*min_width = burst_length * 8 / bpp;

	DRM_DEBUG_KMS("%s:min_width[%d]\n", __func__, *min_width);

	return 0;
}

/* FIXME: need to check pixel align */
static int fimd_check_align_width(struct fimd_win_data *win_data, int width)
{
	int pixel_align, bpp, bytes_align = 8;

	bpp = (win_data->bpp >> 3);
	pixel_align = bytes_align / bpp;

	DRM_DEBUG_KMS("%s:width[%d]pixel_align[%d]\n",
		__func__, width, pixel_align);

	if (width % pixel_align) {
		DRM_ERROR("failed to set width[%d]pixel_align[%d]\n",
			width, pixel_align);
		return -EINVAL;
	}

	return 0;
}

static int fimd_check_restriction(struct fimd_win_data *win_data)
{
	u32 min_width;
	int ret;

	DRM_DEBUG_KMS("%s:line_size[%d]buf_offsize[%d]\n",
		__func__, win_data->line_size, win_data->buf_offsize);
	DRM_DEBUG_KMS("%s:offset_x[%d]ovl_width[%d]\n",
		__func__, win_data->offset_x, win_data->ovl_width);

	/*
	 * You should align the sum of PAGEWIDTH_F and
	 * OFFSIZE_F double-word (8 byte) boundary.
	 */
	if (VIDW0nADD2_DWORD_CHECK(win_data->line_size,
	    win_data->buf_offsize)) {
		DRM_ERROR("failed to check double-word align.\n");
		return -EINVAL;
	}

	ret = fimd_check_align_width(win_data, win_data->ovl_width);
	if (ret) {
		DRM_ERROR("failed to check pixel align.\n");
		return -EINVAL;
	}

	ret = fimd_check_min_width(win_data, &min_width);
	if (ret) {
		DRM_ERROR("failed to check minimum width.\n");
		return -EINVAL;
	}

	if (win_data->ovl_width < min_width) {
		DRM_ERROR("invalid minimum %d width.\n", win_data->ovl_width);
		return -EINVAL;
	}

	return 0;
}

static inline void fimd_win_channel_ctrl(struct fimd_context *ctx, int win,
		bool enable)
{
	unsigned long val;

	val = readl(ctx->regs + WINCON(win));
	val &= ~WINCONx_ENWIN;
	if (enable)
		val |= WINCONx_ENWIN;
	writel(val, ctx->regs + WINCON(win));
}

static inline void fimd_win_shadow_ctrl(struct fimd_context *ctx,
		int win, int flags)
{
	struct fimd_win_data *win_data;
	unsigned long val;

	win_data = &ctx->win_data[win];

	val = readl(ctx->regs + SHADOWCON);
	val &= ~SHADOWCON_WINx_PROTECT(win);

	if (flags & FIMD_SC_PROTECT)
		val |= SHADOWCON_WINx_PROTECT(win);
	else {
		if (!(flags & FIMD_SC_BYPASS_CH_CTRL)) {
			val &= ~(SHADOWCON_CHx_ENABLE(win) |
				SHADOWCON_CHx_LOCAL_ENABLE(win));

			if (flags & FIMD_SC_CH_ENABLE) {
				val |= SHADOWCON_CHx_ENABLE(win);
				if (win_data->local_path)
					val |= SHADOWCON_CHx_LOCAL_ENABLE(win);
			}
		}
	}

	writel(val, ctx->regs + SHADOWCON);
}

static void fimd_win_commit(struct device *dev, int zpos)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	struct drm_device *drm_dev = ctx->subdrv.drm_dev;
	struct fimd_win_data *win_data;
	int win = zpos;
	unsigned long val, alpha, size;
	u32 br_x, br_y;
	int dpms;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (win == DEFAULT_ZPOS)
		win = ctx->default_win;

	if (win < 0 || win > WINDOWS_NR)
		return;

	win_data = &ctx->win_data[win];

	if (ctx->i80_if) {
		if (!atomic_read(&ctx->do_apply) &&
			!atomic_read(&ctx->partial_requested))
			fimd_prepare_i80_access(ctx, __func__);
	}

	dpms = fimd_get_dpms(ctx, 0);

	if (dpms > 0) {
		DRM_ERROR("invalid:cur_dpms[%d]\n", dpms);
		return;
	}

	if (fimd_check_restriction(win_data)) {
		DRM_ERROR("failed to check %d window restriction.\n", win);
		return;
	}

	br_x = win_data->offset_x + win_data->ovl_width;
	if (br_x)
		br_x--;

	br_y = win_data->offset_y + win_data->ovl_height;
	if (br_y)
		br_y--;

	/*
	 * SHADOWCON register is used for enabling timing.
	 *
	 * for example, once only width value of a register is set,
	 * if the dma is started then fimd hardware could malfunction so
	 * with protect window setting, the register fields with prefix '_F'
	 * wouldn't be updated at vsync also but updated once unprotect window
	 * is set.
	 */

	/* protect windows */
	fimd_win_shadow_ctrl(ctx, win, FIMD_SC_PROTECT);

	/* buffer start address */
	val = (unsigned long)win_data->dma_addr;
	writel(val, ctx->regs + VIDWx_BUF_START(win, 0));

	/* buffer end address */
	size = win_data->fb_width * win_data->ovl_height * (win_data->bpp >> 3);
	val = (unsigned long)(win_data->dma_addr + size);
	writel(val, ctx->regs + VIDWx_BUF_END(win, 0));

	DRM_DEBUG_KMS("start addr = 0x%lx, end addr = 0x%lx, size = 0x%lx\n",
			(unsigned long)win_data->dma_addr, val, size);
	DRM_DEBUG_KMS("ovl_width = %d, ovl_height = %d\n",
			win_data->ovl_width, win_data->ovl_height);

	/* buffer size */
	val = VIDW_BUF_SIZE_OFFSET(win_data->buf_offsize) |
		VIDW_BUF_SIZE_PAGEWIDTH(win_data->line_size) |
		VIDW_BUF_SIZE_OFFSET_E(win_data->buf_offsize) |
		VIDW_BUF_SIZE_PAGEWIDTH_E(win_data->line_size);
	writel(val, ctx->regs + VIDWx_BUF_SIZE(win, 0));

	DRM_DEBUG_KMS("%s:win[%d]x[%d]y[%d]w[%d]h[%d]\n", __func__,
		win, win_data->offset_x, win_data->offset_y,
		win_data->ovl_width, win_data->ovl_height);

	/* OSD position */
	val = VIDOSDxA_TOPLEFT_X(win_data->offset_x) |
		VIDOSDxA_TOPLEFT_Y(win_data->offset_y) |
		VIDOSDxA_TOPLEFT_X_E(win_data->offset_x) |
		VIDOSDxA_TOPLEFT_Y_E(win_data->offset_y);
	writel(val, ctx->regs + VIDOSD_A(win));

	val = VIDOSDxB_BOTRIGHT_X(br_x) | VIDOSDxB_BOTRIGHT_Y(br_y) |
		VIDOSDxB_BOTRIGHT_X_E(br_x) | VIDOSDxB_BOTRIGHT_Y_E(br_y);

	writel(val, ctx->regs + VIDOSD_B(win));

	DRM_DEBUG_KMS("osd pos: tx = %d, ty = %d, bx = %d, by = %d\n",
			win_data->offset_x, win_data->offset_y, br_x, br_y);

	/* hardware window 0 doesn't support alpha channel. */
	if (win != 0) {
		/* OSD alpha */
		alpha = VIDOSDxC_ALPHA0_R_H(0x0) |
			VIDOSDxC_ALPHA0_G_H(0x0) |
			VIDOSDxC_ALPHA0_B_H(0x0) |
			VIDOSDxC_ALPHA1_R_H(0xff) |
			VIDOSDxC_ALPHA1_G_H(0xff) |
			VIDOSDxC_ALPHA1_B_H(0xff);

		writel(alpha, ctx->regs + VIDOSD_C(win));
	}

	/* OSD size */
	if (win != 3 && win != 4) {
		u32 offset = VIDOSD_D(win);
		if (win == 0)
			offset = VIDOSD_C_SIZE_W0;
		val = win_data->ovl_width * win_data->ovl_height;
		writel(val, ctx->regs + offset);

		DRM_DEBUG_KMS("osd size = 0x%x\n", (unsigned int)val);
	}

	fimd_win_set_pixfmt(dev, win);

	/* hardware window 0 doesn't support color key. */
	if (win != 0)
		fimd_win_set_colkey(dev, win);

	if (is_drm_iommu_supported(drm_dev) && (!ctx->iommu_on)) {
		/* wincon */
		fimd_win_channel_ctrl(ctx, win, false);

		/* unprotect windows */
		fimd_win_shadow_ctrl(ctx, win, FIMD_SC_BYPASS_CH_CTRL);
	} else {
		/* wincon */
		fimd_win_channel_ctrl(ctx, win, true);

		/* Enable DMA channel and unprotect windows */
		fimd_win_shadow_ctrl(ctx, win, FIMD_SC_CH_ENABLE);
	}

	win_data->enabled = true;

	if (ctx->i80_if) {
		unsigned long flags;

		spin_lock_irqsave(&ctx->win_updated_lock, flags);

		if (!atomic_read(&ctx->do_apply))
			atomic_set(&ctx->win_updated, 1);

		spin_unlock_irqrestore(&ctx->win_updated_lock, flags);
	}
}

static int fimd_iommu_start(struct fimd_context *ctx)
{
	struct device *client = ctx->subdrv.dev;
	struct drm_device *drm_dev = ctx->subdrv.drm_dev;
	int ret;

	/* check if iommu is enabled or not and first call or not. */
	if (!is_drm_iommu_supported(drm_dev) || ctx->iommu_on)
		return 0;

	fimd_wait_for_completed_signal(client);

	/* enable fimd's iommu. */
	ret = drm_iommu_attach_device(drm_dev, client);
	if (ret < 0) {
		DRM_ERROR("failed to enable iommu.\n");
		return ret;
	}

	ctx->iommu_on = true;

	return 0;
}

static void fimd_win_enable(struct device *dev, int zpos)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	struct drm_device *drm_dev = ctx->subdrv.drm_dev;
	int win = zpos;
	int dpms;


	if (win == DEFAULT_ZPOS)
		win = ctx->default_win;

	if (win < 0 || win >= WINDOWS_NR)
		return;

	if (ctx->i80_if)
		fimd_prepare_i80_access(ctx, __func__);

	dpms = fimd_get_dpms(ctx, 0);

	DRM_INFO("%s:win[%d]cur_dpms[%d]\n", __func__,
		win, dpms);

	if (dpms > 0) {
		DRM_ERROR("invalid:cur_dpms[%d]\n", dpms);
		return;
	}

	 if (is_drm_iommu_supported(drm_dev) && (!ctx->iommu_on)) {
		/* now try to change it into iommu mode. */
		fimd_iommu_start(ctx);

		/* protect windows */
		fimd_win_shadow_ctrl(ctx, win, FIMD_SC_PROTECT);

		/* wincon */
		fimd_win_channel_ctrl(ctx, win, true);

		/* Enable DMA channel and unprotect windows */
		fimd_win_shadow_ctrl(ctx, win, FIMD_SC_CH_ENABLE);
	 }
}

static void fimd_win_disable(struct device *dev, int zpos)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	struct fimd_win_data *win_data;
	int win = zpos;
	int dpms;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (win == DEFAULT_ZPOS)
		win = ctx->default_win;

	if (win < 0 || win >= WINDOWS_NR)
		return;

	win_data = &ctx->win_data[win];

	if (ctx->i80_if)
		fimd_prepare_i80_access(ctx, __func__);

	dpms = fimd_get_dpms(ctx, 0);

	DRM_INFO("%s:win[%d]cur_dpms[%d]\n", __func__,
		win, dpms);

	if (dpms > 0) {
		DRM_ERROR("invalid:cur_dpms[%d]\n", dpms);
		goto out;
	}

	/* protect windows */
	fimd_win_shadow_ctrl(ctx, win, FIMD_SC_PROTECT);

	/* wincon */
	fimd_win_channel_ctrl(ctx, win, false);

	/* Disable DMA channel and unprotect windows */
	fimd_win_shadow_ctrl(ctx, win, FIMD_SC_CH_DISABLE);

out:
	win_data->enabled = false;
}

static struct exynos_drm_overlay_ops fimd_overlay_ops = {
	.mode_set = fimd_win_mode_set,
	.commit = fimd_win_commit,
	.enable = fimd_win_enable,
	.disable = fimd_win_disable,
	.partial_resolution = fimd_partial_resolution,
	.request_partial_update = fimd_request_partial_update,
	.adjust_partial_region = fimd_adjust_partial_region,
};

static struct exynos_drm_manager fimd_manager = {
	.pipe		= -1,
	.ops		= &fimd_manager_ops,
	.overlay_ops	= &fimd_overlay_ops,
	.display_ops	= &fimd_display_ops,
};

static irqreturn_t fimd_irq_handler(int irq, void *dev_id)
{
	struct fimd_context *ctx = (struct fimd_context *)dev_id;
	struct exynos_drm_subdrv *subdrv = &ctx->subdrv;
	struct exynos_drm_manager *manager = subdrv->manager;
	u32 val, clear_bit;

	val = readl(ctx->regs + VIDINTCON1);

	DRM_DEBUG_KMS("[%s]:val[0x%08x]\n", __func__, val);

	clear_bit = ctx->i80_if ? VIDINTCON1_INT_I180 : VIDINTCON1_INT_FRAME;

	if (val & clear_bit)
		writel(clear_bit, ctx->regs + VIDINTCON1);

	/* check the crtc is detached already from encoder */
	if (manager->pipe < 0) {
		DRM_ERROR("crtc is detached\n");
		goto out;
	}

	if (!ctx->i80_if) {
		drm_handle_vblank(ctx->drm_dev, ctx->pipe);
		exynos_drm_crtc_finish_pageflip(ctx->drm_dev, ctx->pipe);
	} else {
		/* unset i80 fame done interrupt */
		val = readl(ctx->regs + VIDINTCON0);
		val &= ~(VIDINTCON0_INT_I80IFDONE | VIDINTCON0_INT_SYSMAINCON);

		if (ctx->dbg_cnt) {
			DRM_INFO("IRQ[0x%x->0x%x]c[%d]\n",
				readl(ctx->regs + VIDINTCON0), val,
				ctx->dbg_cnt);
		}

		writel(val, ctx->regs + VIDINTCON0);

		/* exit triggering mode */
		atomic_set(&ctx->triggering, 0);
	}

	/* set wait vsync event to zero and wake up queue. */
	if (atomic_read(&ctx->wait_vsync_event)) {
		atomic_set(&ctx->wait_vsync_event, 0);
		DRM_WAKEUP(&ctx->wait_vsync_queue);
	}

out:
	return IRQ_HANDLED;
}

static void fimd_clear_win(struct fimd_context *ctx, int win)
{
	u32 val;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	writel(0, ctx->regs + WINCON(win));
	writel(0, ctx->regs + VIDOSD_A(win));
	writel(0, ctx->regs + VIDOSD_B(win));
	writel(0, ctx->regs + VIDOSD_C(win));

	if (win == 1 || win == 2)
		writel(0, ctx->regs + VIDOSD_D(win));

	writel(0, ctx->regs + VIDWx_BUF_START(win, 0));
	writel(0, ctx->regs + VIDWx_BUF_END(win, 0));

	val = readl(ctx->regs + SHADOWCON);
	val &= ~(SHADOWCON_CHx_ENABLE(win) |
		SHADOWCON_WINx_PROTECT(win));
	writel(val, ctx->regs + SHADOWCON);
}

static int fimd_power_on(struct fimd_context *ctx, bool enable)
{
	struct exynos_drm_subdrv *subdrv = &ctx->subdrv;
	struct device *dev = subdrv->dev;
	int ret = 0;

	DRM_INFO("fimd_pm[%d]tr[%d]\n",
		enable, atomic_read(&ctx->triggering));

	if (enable != false && enable != true) {
		ret = -EINVAL;
		goto err;
	}

	if (enable) {
		int ret;

		ret = clk_enable(ctx->bus_clk);
		if (ret < 0)
			goto err;

		ret = clk_enable(ctx->lcd_clk);
		if  (ret < 0) {
			clk_disable(ctx->bus_clk);
			goto err;
		}

		ctx->dpms = DRM_MODE_DPMS_ON;

		/* if vblank was enabled status, enable it again. */
		fimd_enable_vblank(ctx->dev);

		if (!atomic_read(&ctx->te_irq_on)) {
			enable_irq(gpio_to_irq(ctx->te_gpio));
			DRM_INFO("te:irq_on\n");
			atomic_set(&ctx->te_irq_on, 1);
		}

		fimd_apply(dev);
	} else {
		fimd_disable_vblank(ctx->dev);

		if (atomic_read(&ctx->te_irq_on)) {
			disable_irq(gpio_to_irq(ctx->te_gpio));
			DRM_INFO("te:irq_off\n");
			atomic_set(&ctx->te_irq_on, 0);
		}
	}

err:
	DRM_INFO("fimd_pm:done[%d]ret[%d]tr[%d]\n",
		enable, ret, atomic_read(&ctx->triggering));

	return ret;
}

static int fimd_configure_clocks(struct fimd_context *ctx, struct device *dev)
{
	struct exynos_drm_panel_info *panel = ctx->panel;
	struct fb_videomode *timing = &panel->timing;
	unsigned long clk;
	int ret;

	ctx->bus_clk = clk_get(dev, "fimd");
	if (IS_ERR(ctx->bus_clk)) {
		dev_err(dev, "failed to get bus clock\n");
		ret = PTR_ERR(ctx->bus_clk);
		goto err_disable_block_clk;
	}

	ret = clk_prepare(ctx->bus_clk);
	if (ret) {
		dev_err(dev, "failed to prepare bus clock\n");
		goto err_disable_block_clk;
	}

	ctx->lcd_clk = clk_get(dev, "sclk_fimd");
	if (IS_ERR(ctx->lcd_clk)) {
		dev_err(dev, "failed to get lcd clock\n");
		ret = PTR_ERR(ctx->lcd_clk);
		goto err_unprepare_bus_clk;
	}

	ret = clk_prepare(ctx->lcd_clk);
	if (ret) {
		dev_err(dev, "failed to prepare lcd clock\n");
		goto err_unprepare_bus_clk;
	}

	clk = clk_get_rate(ctx->lcd_clk);
	if (clk == 0) {
		dev_err(dev, "error getting sclk_fimd clock rate\n");
		ret = -EINVAL;
		goto err_unprepare_lcd_clk;
	}

	if (timing->pixclock == 0) {
		unsigned long c;
		if (ctx->i80_if) {
			c = timing->xres * timing->yres *
			    (timing->cs_setup_time +
			    timing->wr_setup_time + timing->wr_act_time +
			    timing->wr_hold_time + 1);

			/*
			 * Basically, the refresh rate of TE and vsync is 60Hz.
			 * In case of using i80 lcd panel, we need to do fimd
			 * trigger so that graphics ram in the lcd panel to be
			 * updated.
			 *
			 * And, we do fimd trigger every time TE interrupt
			 * handler occurs to resolve tearing issue on the lcd
			 * panel. However, this way doesn't avoid the tearing
			 * issue because fimd i80 frame done interrupt doesn't
			 * occur since fimd trigger before next TE interrupt.
			 *
			 * So makes video clock speed up two times so that the
			 * fimd i80 frame done interrupt can occur prior to
			 * next TE interrupt.
			 */
			c *= 2;
		} else {
			c = timing->xres + timing->left_margin + timing->right_margin +
			    timing->hsync_len;
			c *= timing->yres + timing->upper_margin + timing->lower_margin +
			     timing->vsync_len;
		}

		timing->pixclock = c * FIMD_HIGH_FRAMERATE;
		if (timing->pixclock == 0) {
			dev_err(dev, "incorrect display timings\n");
			ret = -EINVAL;
			goto err_unprepare_lcd_clk;
		}
	}

	ctx->clkdiv = DIV_ROUND_UP(clk, timing->pixclock);
	if (ctx->clkdiv > 256) {
		dev_warn(dev, "calculated pixel clock divider too high (%u), lowered to 256\n",
			 ctx->clkdiv);
		ctx->clkdiv = 256;
	}
	timing->pixclock = clk / ctx->clkdiv;
	DRM_INFO("%s:pixel clock = %d, clkdiv = %d\n", __func__, timing->pixclock,
		       ctx->clkdiv);

	return 0;

err_unprepare_lcd_clk:
	clk_unprepare(ctx->lcd_clk);
err_unprepare_bus_clk:
	clk_unprepare(ctx->bus_clk);
err_disable_block_clk:

	return ret;
}

static struct clk *fimd_get_clk(struct dispfreq_device *dfd)
{
	struct fimd_context *ctx = dispfreq_get_data(dfd);
	struct clk *clk;

	clk = ctx->lcd_clk;

	return clk;
}

static int fimd_set_clk(struct dispfreq_device *dfd,
		struct clksrc_clk *clksrc, int div)
{
	struct fimd_context *ctx = dispfreq_get_data(dfd);
	u32 mask = reg_mask(clksrc->reg_div.shift, clksrc->reg_div.size);
	u32 reg =  __raw_readl(clksrc->reg_div.reg);
	unsigned int val;
	int dpms = fimd_get_dpms(ctx, 0);

	if ((reg & mask) == (div & mask)) {
		DRM_ERROR("div is same as reg.\n");
		return -EINVAL;
	}

	if (dpms > 0) {
		DRM_ERROR("invalid:cur_dpms[%d]\n", dpms);
		goto out;
	}

	reg &= ~(0xff);
	reg |= div;

	while (1) {
		val = (__raw_readl(ctx->regs + VIDCON1) &
			VIDCON1_VSTATUS_MASK);
		if (val == VIDCON1_VSTATUS_VSYNC) {
			writel(reg, clksrc->reg_div.reg);
			break;
		}
	}

out:
	return 0;
}

static int fimd_get_div(struct dispfreq_device *dfd)
{
	struct fimd_context *ctx = dispfreq_get_data(dfd);
	unsigned int val = -1;
	int dpms = fimd_get_dpms(ctx, 0);

	if (dpms > 0) {
		DRM_ERROR("invalid:cur_dpms[%d]\n", dpms);
		goto out;
	}

	if (ctx->regs) {
		val = readl(ctx->regs + VIDCON0);
		val &= VIDCON0_CLKVAL_F_MASK;
		val >>= VIDCON0_CLKVAL_F_SHIFT;
	}

out:
	return val;
}

static u32 fimd_get_refresh(struct dispfreq_device *dfd)
{
	struct dispfreq_properties *props = &dfd->props;
	struct fb_videomode *timing = props->timing;

	return timing->refresh;
}

static int fimd_get_pm_state(struct dispfreq_device *dfd)
{
	struct fimd_context *ctx = dispfreq_get_data(dfd);

	if (fimd_get_dpms(ctx, 0) == DRM_MODE_DPMS_ON)
		return 1;
	else
		return 0;
}

static const struct dispfreq_ops fimd_dispfreq_ops = {
	.get_clk = fimd_get_clk,
	.set_clk = fimd_set_clk,
	.get_fimd_div = fimd_get_div,
	.get_refresh = fimd_get_refresh,
	.get_pm_state = fimd_get_pm_state,
};

#ifdef CONFIG_DRM_EXYNOS_IPP
static void fimd_set_writeback(struct fimd_context *ctx, int enable,
		unsigned int refresh)
{
	u32 vidoutcon = readl(ctx->regs + VIDOUT_CON);
	u32 vidcon2 = readl(ctx->regs + VIDCON2);

	DRM_INFO("%s:wb[%d]refresh[%d]\n",
		__func__, enable, refresh);

	vidoutcon &= ~VIDOUT_CON_F_MASK;
	vidcon2 &= ~(VIDCON2_WB_MASK |
			VIDCON2_WB_SKIP_MASK |
			VIDCON2_TVFORMATSEL_HW_SW_MASK);

	if (enable) {
		vidoutcon |= VIDOUT_CON_WB;
		if (ctx->i80_if)
			vidoutcon |= VIDOUT_CON_F_I80_LDI0;
		vidcon2 |= (VIDCON2_WB_ENABLE |
				VIDCON2_TVFORMATSEL_SW);

		if (refresh >= 60 || refresh == 0)
			DRM_INFO("%s:refresh[%d],forced set to 60hz.\n",
				__func__, refresh);
		else if (refresh >= 30)
			vidcon2 |= VIDCON2_WB_SKIP_1_2;
		else if (refresh >= 20)
			vidcon2 |= VIDCON2_WB_SKIP_1_3;
		else if (refresh >= 15)
			vidcon2 |= VIDCON2_WB_SKIP_1_4;
		else
			vidcon2 |= VIDCON2_WB_SKIP_1_5;
	} else {
		if (ctx->i80_if)
			vidoutcon |= VIDOUT_CON_F_I80_LDI0;
		else
			vidoutcon |= VIDOUT_CON_RGB;
		vidcon2 |= VIDCON2_WB_DISABLE;
	}

	writel(vidoutcon, ctx->regs + VIDOUT_CON);
	writel(vidcon2, ctx->regs + VIDCON2);
}

static void fimd_set_output(struct fimd_context *ctx,
		struct exynos_drm_overlay *overlay)
{
	struct fimd_win_data *win_data;
	struct device *dev = ctx->subdrv.dev;

	DRM_DEBUG_KMS("%s:zpos[%d]activated[%d]\n", __func__,
		overlay->zpos, overlay->activated);

	win_data = &ctx->win_data[overlay->zpos];

	if (overlay->activated) {
		fimd_win_mode_set(dev, overlay);
		fimd_win_commit(dev, overlay->zpos);
		fimd_win_enable(dev, overlay->zpos);
	} else
		fimd_win_disable(dev, overlay->zpos);
}

static int fimd_notifier_ctrl(struct notifier_block *this,
			unsigned long event, void *_data)
{
	struct fimd_context *ctx = container_of(this,
				struct fimd_context, nb_ctrl);

	switch (event) {
	case IPP_GET_LCD_WIDTH: {
		struct exynos_drm_panel_info *panel = ctx->panel;
		struct fb_videomode *timing = &panel->timing;
		int *width = (int *)_data;

		*width = timing->xres;
	}
		break;
	case IPP_GET_LCD_HEIGHT: {
		struct exynos_drm_panel_info *panel = ctx->panel;
		struct fb_videomode *timing = &panel->timing;
		int *height = (int *)_data;

		*height = timing->yres;
	}
		break;
	case IPP_SET_WRITEBACK: {
		struct drm_exynos_ipp_set_wb *set_wb =
			(struct drm_exynos_ipp_set_wb *)_data;
		unsigned int refresh = set_wb->refresh;
		int enable = *((int *)&set_wb->enable);

		fimd_set_writeback(ctx, enable, refresh);
	}
		break;
	case IPP_SET_OUTPUT: {
		struct exynos_drm_overlay *overlay =
			(struct exynos_drm_overlay *)_data;

		fimd_set_output(ctx, overlay);
	}
		break;
	default:
		/* ToDo : for checking use case */
		DRM_INFO("%s:event[0x%x]\n", __func__, (unsigned int)event);
		break;
	}

	return NOTIFY_DONE;
}
#endif

#ifdef CONFIG_DRM_EXYNOS_DBG
static int fimd_read_reg(struct fimd_context *ctx, char *buf)
{
	struct fimd_driver_data *driver_data = ctx->driver_data;
	struct resource *res = ctx->regs_res;
	int i, pos = 0;
	u32 cfg;

	pos += sprintf(buf+pos, "0x%.8x | ", res->start);
	for (i = 1; i < FIMD_MAX_REG + 1; i++) {
		cfg = readl(ctx->regs + ((i-1) * sizeof(u32)));
		pos += sprintf(buf+pos, "0x%.8x ", cfg);
		if (i % 4 == 0)
			pos += sprintf(buf+pos, "\n0x%.8x | ",
				res->start + (i * sizeof(u32)));
	}
	pos += sprintf(buf+pos, "\n");

	if (driver_data->timing_base) {
		pos += sprintf(buf+pos, "0x%.8x | ", res->start
			+ driver_data->timing_base);
		for (i = 1; i < FIMD_MAX_REG + 1; i++) {
			cfg = readl(ctx->regs + ctx->driver_data->timing_base
				+ ((i-1) * sizeof(u32)));
			pos += sprintf(buf+pos, "0x%.8x ", cfg);
			if (i % 4 == 0)
				pos += sprintf(buf+pos, "\n0x%.8x | ",
					res->start + (i * sizeof(u32)));
		}

		pos += sprintf(buf+pos, "\n");
	}

	return pos;
}

static ssize_t show_read_reg(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct fimd_context *ctx = get_fimd_context(dev);

	if (!ctx->regs) {
		dev_err(dev, "failed to get current register.\n");
		return -EINVAL;
	}

	return fimd_read_reg(ctx, buf);
}

static ssize_t show_overlay_info(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fimd_context *ctx = get_fimd_context(dev);
	struct fimd_win_data *win_data;
	int win, pos = 0;

	for (win = 0; win < WINDOWS_NR; win++) {
		win_data = &ctx->win_data[win];

		if (win_data->enabled) {
			pos += sprintf(buf+pos, "Overlay %d: ", win);
			pos += sprintf(buf+pos, "x[%d],y[%d],w[%d],h[%d],",
				win_data->offset_x, win_data->offset_y,
				win_data->ovl_width, win_data->ovl_height);
			pos += sprintf(buf+pos, "fb_width[%d],fb_height[%d]\n",
				win_data->fb_width, win_data->fb_height);
		}
	}

	return pos;
}

static struct device_attribute device_attrs[] = {
	__ATTR(read_reg, S_IRUGO, show_read_reg, NULL),
	__ATTR(overlay_info, S_IRUGO, show_overlay_info, NULL),
};
#endif

static int __devinit fimd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fimd_context *ctx;
	struct exynos_drm_subdrv *subdrv;
	struct exynos_drm_fimd_pdata *pdata;
	struct exynos_drm_panel_info *panel;
	struct dispfreq_properties props;
	struct fb_videomode *timing;
	struct resource *res;
	int win;
	int ret = -EINVAL;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(dev, "no platform data specified\n");
		return -EINVAL;
	}

	panel = &pdata->panel;
	if (!panel) {
		dev_err(dev, "panel is null.\n");
		return -EINVAL;
	}

	timing = &panel->timing;
	if (!timing) {
		dev_err(dev, "timing is null.\n");
		return -EINVAL;
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	pdata->i80ifcon = (LCD_CS_SETUP(timing->cs_setup_time)
					| LCD_WR_SETUP(timing->wr_setup_time)
					| LCD_WR_ACT(timing->wr_act_time)
					| LCD_WR_HOLD(timing->wr_hold_time)
					| LCD_WR_RS_POL(timing->rs_pol));

	ctx->te_gpio = pdata->te_gpio;
	if (gpio_request_one(ctx->te_gpio, GPIOF_IN, "lcd-te"))
		dev_err(dev, "failed to gpio_request_one\n");

	ctx->disp_bus_pdev = pdata->disp_bus_pdev;
	pdata->trigger = fimd_trigger;
	pdata->wait_for_frame_done = fimd_wait_for_completed_signal;
	pdata->stop_trigger = fimd_stop_trigger;
	pdata->set_runtime_activate = fimd_set_runtime_activate;
	pdata->update_panel_refresh = fimd_update_panel_refresh;
#ifdef CONFIG_EXYNOS_SMIES
#ifdef CONFIG_EXYNOS_SMIES_DEFALUT_ENABLE
	pdata->set_smies_mode = fimd_set_smies_mode;
	atomic_set(&ctx->smies_active, 1);
#else
	pdata->set_smies_activate = fimd_set_smies_activate;
#endif
#endif
	ctx->bus_clk = clk_get(dev, "fimd");
	if (IS_ERR(ctx->bus_clk)) {
		dev_err(dev, "failed to get bus clock\n");
		ret = PTR_ERR(ctx->bus_clk);
		goto err_clk_get;
	}

	ctx->lcd_clk = clk_get(dev, "sclk_fimd");
	if (IS_ERR(ctx->lcd_clk)) {
		dev_err(dev, "failed to get lcd clock\n");
		ret = PTR_ERR(ctx->lcd_clk);
		goto err_bus_clk;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "failed to find registers\n");
		ret = -ENOENT;
		goto err_clk;
	}

	ctx->regs_res = request_mem_region(res->start, resource_size(res),
					   dev_name(dev));
	if (!ctx->regs_res) {
		dev_err(dev, "failed to claim register region\n");
		ret = -ENOENT;
		goto err_clk;
	}

	ctx->regs = ioremap(res->start, resource_size(res));
	if (!ctx->regs) {
		dev_err(dev, "failed to map registers\n");
		ret = -ENXIO;
		goto err_req_region_io;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
						timing->i80en ? "lcd_sys" : "vsync");
	if (!res) {
		dev_err(dev, "irq i80_if irq failed.\n");
		goto err_req_region_irq;
	}

	ctx->irq = res->start;
	ret = request_irq(ctx->irq, fimd_irq_handler, 0, "drm_fimd", ctx);
	if (ret < 0) {
		dev_err(dev, "irq request failed.\n");
		goto err_req_region_irq;
	}

	ctx->driver_data = drm_fimd_get_driver_data(pdev);
	ctx->vidcon0 = pdata->vidcon0;
	ctx->vidcon1 = pdata->vidcon1;
	ctx->i80ifcon = pdata->i80ifcon;
	ctx->default_win = pdata->default_win;
	ctx->panel = panel;
	ctx->i80_if = timing->i80en;
	ctx->dev = &pdev->dev;
#ifdef CONFIG_EXYNOS_SMIES
	ctx->smies_device = pdata->smies_device;
	ctx->smies_on = pdata->smies_on;
	ctx->smies_off=  pdata->smies_off;
	ctx->smies_mode=  pdata->smies_mode;
#endif
	DRM_INIT_WAITQUEUE(&ctx->wait_vsync_queue);
	DRM_INIT_WAITQUEUE(&ctx->wait_te_queue);
	atomic_set(&ctx->wait_vsync_event, 0);
	atomic_set(&ctx->wait_te_event, 0);

	props.timing = &panel->timing;
	props.refresh = props.timing->refresh;
	props.max_refresh = pdata->max_refresh;
	props.min_refresh = pdata->min_refresh;

#ifdef CONFIG_DISPFREQ_CLASS_DEVICE
	ctx->dfd = dispfreq_device_register("exynos", &pdev->dev, ctx,
			&fimd_dispfreq_ops, &props);
	if (IS_ERR(ctx->dfd)) {
		dev_err(dev, "failed to register dispfreq.\n");
		ret = PTR_ERR(ctx->dfd);
		goto err_req_irq;
	}
#endif

	subdrv = &ctx->subdrv;

	subdrv->dev = dev;
	subdrv->manager = &fimd_manager;
	subdrv->probe = fimd_subdrv_probe;
	subdrv->remove = fimd_subdrv_remove;

#ifdef CONFIG_DRM_EXYNOS_IPP
	ctx->nb_ctrl.notifier_call = fimd_notifier_ctrl;
	ret = exynos_drm_ippnb_register(&ctx->nb_ctrl);
	if (ret) {
		dev_err(dev, "could not register fimd notify callback\n");
		goto err_req_irq;
	}
#endif

#ifdef CONFIG_DRM_EXYNOS_DBG
	exynos_drm_dbg_register(&pdev->dev, device_attrs,
			ARRAY_SIZE(device_attrs), ctx->regs);
	if (ctx->driver_data->timing_base)
		exynos_drm_dbg_register(&pdev->dev, NULL, 0,
				ctx->regs+ctx->driver_data->timing_base);
#endif

	ctx->pm_wq= create_singlethread_workqueue("fimd_pm");
	if (!ctx->pm_wq) {
		DRM_ERROR("failed to create workq.\n");
		goto err_req_irq;
	}

	INIT_WORK(&ctx->pm_work, fimd_activate_work);

	mutex_init(&ctx->lock);
	spin_lock_init(&ctx->win_updated_lock);

	platform_set_drvdata(pdev, ctx);

	atomic_set(&ctx->te_irq_on, 0);

	irq_set_status_flags(gpio_to_irq(ctx->te_gpio), IRQ_NOAUTOEN);

	ret = devm_request_irq(dev, gpio_to_irq(ctx->te_gpio),
				panel_te_interrupt,
				IRQF_DISABLED | IRQF_TRIGGER_RISING,
				"TE", ctx);
	if (ret < 0) {
		dev_err(dev, "failed to request te irq.\n");
		return ret;
	}

	ctx->dpms = DRM_MODE_DPMS_OFF;
	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);
	fimd_power_on(ctx, true);

	ret = fimd_configure_clocks(ctx, dev);
	if (ret)
		return ret;

	for (win = 0; win < WINDOWS_NR; win++)
		fimd_clear_win(ctx, win);

	exynos_drm_subdrv_register(subdrv);

	if (!lpcharge)
		fimd_power_gate_set(ctx, true);
	else
		DRM_INFO("%s:bypass to set power gate:lpcharge\n",
			__func__);

	return 0;

#ifdef CONFIG_DISPFREQ_CLASS_DEVICE
err_req_irq:
	free_irq(ctx->irq, ctx);
#endif
err_req_region_irq:
	iounmap(ctx->regs);

err_req_region_io:
	release_resource(ctx->regs_res);
	kfree(ctx->regs_res);

err_clk:
	clk_disable(ctx->lcd_clk);
	clk_put(ctx->lcd_clk);

err_bus_clk:
	clk_disable(ctx->bus_clk);
	clk_put(ctx->bus_clk);

err_clk_get:
	kfree(ctx);
	return ret;
}

static int __devexit fimd_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fimd_context *ctx = platform_get_drvdata(pdev);
	int dpms = fimd_get_dpms(ctx, 0);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	fimd_power_gate_set(ctx, false);

	exynos_drm_subdrv_unregister(&ctx->subdrv);

#ifdef CONFIG_DRM_EXYNOS_DBG
	exynos_drm_dbg_unregister(&pdev->dev, device_attrs);
	if (ctx->driver_data->timing_base)
		exynos_drm_dbg_unregister(&pdev->dev, NULL);
#endif

#ifdef CONFIG_DRM_EXYNOS_IPP
	exynos_drm_ippnb_unregister(&ctx->nb_ctrl);
#endif

	if (dpms > 0) {
		DRM_ERROR("invalid:cur_dpms[%d]\n", dpms);
		goto out;
	}

	clk_disable(ctx->lcd_clk);
	clk_disable(ctx->bus_clk);

	pm_runtime_set_suspended(dev);

	fimd_power_on(ctx, false);
	pm_runtime_put_sync(dev);

out:
	pm_runtime_disable(dev);

	clk_put(ctx->lcd_clk);
	clk_put(ctx->bus_clk);

	iounmap(ctx->regs);
	release_resource(ctx->regs_res);
	kfree(ctx->regs_res);
	free_irq(ctx->irq, ctx);

	kfree(ctx);

	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static bool fimd_runtime_suspended(struct device *dev)
{
	return pm_runtime_suspended(dev);
}
#endif

static struct platform_device_id fimd_driver_ids[] = {
	{
		.name		= "s3c64xx-fb",
		.driver_data	= (unsigned long)&s3c64xx_fimd_driver_data,
	}, {
		.name		= "exynos3-fb",
		.driver_data	= (unsigned long)&exynos3_fimd_driver_data,
	}, {
		.name		= "exynos4-fb",
		.driver_data	= (unsigned long)&exynos4_fimd_driver_data,
	}, {
		.name		= "exynos5-fb",
		.driver_data	= (unsigned long)&exynos5_fimd_driver_data,
	},
	{},
};

struct platform_driver fimd_driver = {
	.probe		= fimd_probe,
	.remove		= __devexit_p(fimd_remove),
	.id_table       = fimd_driver_ids,
	.driver		= {
		.name	= "exynos3-fb",
		.owner	= THIS_MODULE,
	},
};

