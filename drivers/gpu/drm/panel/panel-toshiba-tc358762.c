#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

struct tc358762_panel {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;

	bool prepared;
	bool enabled;

	struct videomode vm;
	u32 width_mm;
	u32 height_mm;
};

static inline struct tc358762_panel *to_tc358762_panel(struct drm_panel *panel)
{
	return container_of(panel, struct tc358762_panel, base);
}

static void tc358762_gen_write(struct mipi_dsi_device *dsi, const void *data, size_t len)
{
	int ret;

	ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_err(&dsi->dev, "failed to writing gen seq\n");
	}
}

#define tc358762_gen_write_seq(dsi, seq...) \
({\
	static const u8 d[] = { seq };\
	tc358762_gen_write(dsi, d, ARRAY_SIZE(d));\
})

static int tc358762_dsi_init(struct mipi_dsi_device *dsi)
{
	tc358762_gen_write_seq(dsi, 0x10, 0x02, 0x03, 0x00, 0x00, 0x00);//LANE
	tc358762_gen_write_seq(dsi, 0x64, 0x01, 0x0c, 0x00, 0x00, 0x00);//D0S_CLRSIPOCOUNT
	tc358762_gen_write_seq(dsi, 0x68, 0x01, 0x0c, 0x00, 0x00, 0x00);//D1S_CLRSIPOCOUNT
	tc358762_gen_write_seq(dsi, 0x44, 0x01, 0x00, 0x00, 0x00, 0x00);//D0S_ATMR
	tc358762_gen_write_seq(dsi, 0x48, 0x01, 0x00, 0x00, 0x00, 0x00);//D1S_ATMR
	tc358762_gen_write_seq(dsi, 0x14, 0x01, 0x15, 0x00, 0x00, 0x00);//LPTXTIMCNT
	tc358762_gen_write_seq(dsi, 0x50, 0x04, 0x60, 0x00, 0x00, 0x00);//SPICMR/SPICTRL
	tc358762_gen_write_seq(dsi, 0x20, 0x04, 0x52, 0x01, 0x10, 0x00);//PORT/LCDCTRL
	tc358762_gen_write_seq(dsi, 0x24, 0x04, 0x14, 0x00, 0x1a, 0x00);//HSR(20)/HBPR(26)
	tc358762_gen_write_seq(dsi, 0x28, 0x04, 0x20, 0x03, 0xbe, 0x00);//HDISP(800)/HFPR(190)
	tc358762_gen_write_seq(dsi, 0x2c, 0x04, 0x03, 0x00, 0x15, 0x00);//VSR(3)/VBFR(21)
	tc358762_gen_write_seq(dsi, 0x30, 0x04, 0xe0, 0x01, 0x1e, 0x00);//VDISP(480)/VFPR(30)
	tc358762_gen_write_seq(dsi, 0x34, 0x04, 0x01, 0x00, 0x00, 0x00);//VFUEN
	tc358762_gen_write_seq(dsi, 0x64, 0x04, 0x0f, 0x04, 0x00, 0x00);//SYSCTRL
	tc358762_gen_write_seq(dsi, 0x04, 0x01, 0x01, 0x00, 0x00, 0x00);//STARTPPI
	tc358762_gen_write_seq(dsi, 0x04, 0x02, 0x01, 0x00, 0x00, 0x00);//STARTDSI

	msleep(10);
	return 0;
}

int trigger_bridge = 1;
extern void tinker_mcu_screen_power_up(void);
static int tc358762_prepare(struct drm_panel *panel)
{
	struct tc358762_panel *tc = to_tc358762_panel(panel);
	struct mipi_dsi_device *dsi = tc->dsi;

	if (tc->prepared)
		return 0;

	printk("tc358762_prepare\n");

	if(trigger_bridge) {
		pr_info("tinker_mcu_screen_power_up");
		tinker_mcu_screen_power_up();
		trigger_bridge = 0;
	}

	msleep(20);
	printk("tc358762_enable, sleep 20 ms\n");

	tc358762_dsi_init(dsi);

	printk("tc358762_enable, send dsi commend done\n");

	tc->prepared = true;

	return 0;
}

static int tc358762_panel_unprepare(struct drm_panel *panel)
{
	struct tc358762_panel *tc = to_tc358762_panel(panel);

	if (!tc->prepared)
		return 0;

	printk("tc358762_panel_unprepare\n");

	tc->prepared = false;

	return 0;
}

extern int tinker_mcu_set_bright(int bright);
static int tc358762_enable(struct drm_panel *panel)
{
	struct tc358762_panel *tc = to_tc358762_panel(panel);

	if (tc->enabled)
		return 0;

	printk("tc358762_enable\n");

	tinker_mcu_set_bright(0xFF);
	tc->enabled = true;

	return 0;
}

static int tc358762_disable(struct drm_panel *panel)
{
	struct tc358762_panel *tc = to_tc358762_panel(panel);

	if (!tc->enabled)
		return 0;

	printk("tc358762_disable\n");

	tinker_mcu_set_bright(0x00);

	tc->enabled = false;

	return 0;
}

static int tc358762_get_modes(struct drm_panel *panel)
{
	struct tc358762_panel *tc = to_tc358762_panel(panel);
	struct device *dev = &tc->dsi->dev;
	struct drm_connector *connector = panel->connector;
	struct drm_display_mode *mode;
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	u32 *bus_flags = &connector->display_info.bus_flags;
	int ret;

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_DEV_ERROR(dev, "Failed to create display mode!\n");
		return 0;
	}

	drm_display_mode_from_videomode(&tc->vm, mode);
	mode->width_mm = tc->width_mm;
	mode->height_mm = tc->height_mm;
	connector->display_info.width_mm = tc->width_mm;
	connector->display_info.height_mm = tc->height_mm;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	*bus_flags |= DRM_BUS_FLAG_DE_LOW | DRM_BUS_FLAG_PIXDATA_NEGEDGE;

	ret = drm_display_info_set_bus_formats(&connector->display_info,
					       &bus_format, 1);
	if (ret)
		return ret;

	drm_mode_probed_add(panel->connector, mode);

	return 1;
}

static const struct drm_panel_funcs tc358762_funcs = {
	.prepare = tc358762_prepare,
	.unprepare = tc358762_panel_unprepare,
	.enable = tc358762_enable,
	.disable = tc358762_disable,
	.get_modes = tc358762_get_modes,
};

static const struct display_timing tc358762_default_timing = {
	.pixelclock = { 33290000, 33290000, 33290000 },
	.hactive = { 800, 800, 800 },
	.hfront_porch = { 190, 190, 190 },
	.hsync_len = { 20, 20, 20 },
	.hback_porch = { 5, 5, 5 },
	.vactive = { 480, 480, 480 },
	.vfront_porch = { 30, 30, 30 },
	.vsync_len = { 3, 3, 3 },
	.vback_porch = { 20, 20, 20 },
	.flags = DISPLAY_FLAGS_HSYNC_LOW |
		 DISPLAY_FLAGS_VSYNC_LOW |
		 DISPLAY_FLAGS_DE_LOW |
		DISPLAY_FLAGS_PIXDATA_NEGEDGE,
};

static int tc358762_dsi_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *np = dev->of_node;
	struct tc358762_panel *panel;
	int ret;

	panel = devm_kzalloc(&dsi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, panel);

	panel->dsi = dsi;

	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM;

	ret = of_property_read_u32(np, "dsi-lanes", &dsi->lanes);
	if (ret < 0) {
		dev_err(dev, "Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}

	ret = of_get_videomode(np, &panel->vm, 0);
	if (ret < 0)
		videomode_from_timing(&tc358762_default_timing, &panel->vm);

	of_property_read_u32(np, "panel-width-mm", &panel->width_mm);
	of_property_read_u32(np, "panel-height-mm", &panel->height_mm);

	drm_panel_init(&panel->base);
	panel->base.funcs = &tc358762_funcs;
	panel->base.dev = dev;

	ret = drm_panel_add(&panel->base);

	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&panel->base);

	return ret;
}

static int tc358762_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct tc358762_panel *tc = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	ret = tc358762_disable(&tc->base);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "Failed to disable panel (%d)\n", ret);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "Failed to detach from host (%d)\n",
			ret);

	drm_panel_detach(&tc->base);

	if (tc->base.dev)
		drm_panel_remove(&tc->base);

	return 0;
}

static void tc358762_dsi_shutdown(struct mipi_dsi_device *dsi)
{
	struct tc358762_panel *tc = mipi_dsi_get_drvdata(dsi);

	tc358762_disable(&tc->base);
}

static const struct of_device_id dsi_of_match[] = {
	{ .compatible = "asus,tc358762", },
	{ }
};
MODULE_DEVICE_TABLE(of, dsi_of_match);

static struct mipi_dsi_driver tc358762_dsi_driver = {
	.driver = {
		.name = "bridge-tc358762-dsi",
		.of_match_table = dsi_of_match,
	},
	.probe = tc358762_dsi_probe,
	.remove = tc358762_dsi_remove,
	.shutdown = tc358762_dsi_shutdown,
};
module_mipi_dsi_driver(tc358762_dsi_driver);

MODULE_AUTHOR("ASUS");
MODULE_DESCRIPTION("DRM Driver for toshiba tc358762 Bridge");
MODULE_LICENSE("GPL v2");