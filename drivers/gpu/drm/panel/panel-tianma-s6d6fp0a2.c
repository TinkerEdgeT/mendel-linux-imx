#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <uapi/linux/media-bus-format.h>
#include <video/mipi_display.h>
#include <video/videomode.h>

#define CABCCTL1 0xC0
#define PASSWD1 0xF0
#define PASSWD2 0xF1

static const struct display_timing tianma_default_timing = {
	.pixelclock = {0, 166000000, 0},
	.hactive = {0, 1080, 0},
	.hfront_porch = {0, 36, 0},
	.hsync_len = {0, 18, 0},
	.hback_porch = {0, 18, 0},
	.vactive = {0, 2160, 0},
	.vfront_porch = {0, 8, 0},
	.vsync_len = {0, 2, 0},
	.vback_porch = {0, 8, 0},
	.flags = DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH
		 | DISPLAY_FLAGS_DE_HIGH | DISPLAY_FLAGS_PIXDATA_POSEDGE,
};

struct tianma_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *vsp_vsn;
	struct gpio_desc *reset;
	struct backlight_device *backlight;
};

static int tianma_panel_get_modes(struct drm_panel *panel)
{
	struct tianma_panel *tianma =
		container_of(panel, struct tianma_panel, panel);
	struct mipi_dsi_device *dsi = tianma->dsi;
	int ret;
	struct videomode vm;
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;

	struct drm_display_mode *mode;
	mode = drm_mode_create(panel->connector->dev);
	if (!mode) {
		dev_err(&dsi->dev, "Failed drm_mode_create");
		return 0;
	}

	videomode_from_timing(&tianma_default_timing, &vm);
	drm_display_mode_from_videomode(&vm, mode);
	mode->width_mm = 67;
	mode->height_mm = 134;
	panel->connector->display_info.height_mm = mode->height_mm;
	panel->connector->display_info.width_mm = mode->width_mm;
	panel->connector->display_info.bus_flags =
		DRM_BUS_FLAG_PIXDATA_POSEDGE | DRM_BUS_FLAG_DE_HIGH;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	ret = drm_display_info_set_bus_formats(&panel->connector->display_info,
					       &bus_format, 1);
	if (ret) {
		return 0;
	}

	drm_mode_probed_add(panel->connector, mode);

	return 1;
}

static int tianma_panel_prepare(struct drm_panel *panel)
{
	int ret;
	struct tianma_panel *tianma =
		container_of(panel, struct tianma_panel, panel);
	struct mipi_dsi_device *dsi = tianma->dsi;
	uint8_t testkey[2] = {0x5A, 0x5A};
	uint8_t ctrl = 0x2C;
	uint8_t cabcctl1[15] = {0x20, 0x00, 0xFF, 0x01, 0x00, 0x0F, 0x01, 0x24,
				0x0F, 0xFF, 0x30, 0x40, 0x88, 0xE6, 0xA5};
	uint8_t powersave = 0x0;
	uint16_t min_bright = 0x80;

	gpiod_set_value(tianma->reset, 0);
	msleep(100);
	gpiod_set_value(tianma->reset, 1);
	msleep(100);
	gpiod_set_value(tianma->reset, 0);
	msleep(100);

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(&dsi->dev, "Failed to exit sleep mode: %d", ret);
		return ret;
	}
	msleep(120);

	// Unlocks L2 commands, such as CABCCTL.
	ret = mipi_dsi_dcs_write(dsi, PASSWD1, &testkey, sizeof(testkey));
	if (ret < 0) {
		dev_err(&dsi->dev, "Failed to set PASSWD1: %d", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_display_brightness(dsi, 0x0800);
	if (ret < 0) {
		dev_err(&dsi->dev, "Failed to set brightness: %d", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY, &ctrl,
				 sizeof(ctrl));
	if (ret < 0) {
		dev_err(&dsi->dev, "Failed to set control: %d", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_write(dsi, CABCCTL1, cabcctl1, sizeof(cabcctl1));
	if (ret < 0) {
		dev_err(&dsi->dev, "Failed to set cabcctl1: %d", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_WRITE_POWER_SAVE, &powersave,
				 sizeof(powersave));
	if (ret < 0) {
		dev_err(&dsi->dev, "Failed to set powersave: %d", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_SET_CABC_MIN_BRIGHTNESS,
				 &min_bright, sizeof(min_bright));
	if (ret < 0) {
		dev_err(&dsi->dev, "Failed to set min_bright: %d", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(&dsi->dev, "Failed to turn on display: %d", ret);
		return ret;
	}

	return 0;
}

static int tianma_panel_unprepare(struct drm_panel *panel)
{
	struct tianma_panel *tianma =
		container_of(panel, struct tianma_panel, panel);
	struct mipi_dsi_device *dsi = tianma->dsi;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(&dsi->dev, "Could not set display off: %d", ret);
		return ret;
	}
	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(&dsi->dev, "Could not enter sleep mode: %d", ret);
		return ret;
	}
	return 0;
}

static int tianma_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	int ret;

	u16 brightness = 0;
	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0) {
		dev_err(&dsi->dev, "Could not get brightness: %d", ret);
		return ret;
	}
	bl->props.brightness = brightness;
	return brightness & 0xfff;
}

static int tianma_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_set_display_brightness(dsi, bl->props.brightness);
	return ret;
}

static int tianma_panel_enable(struct drm_panel *panel)
{
	struct tianma_panel *tianma =
		container_of(panel, struct tianma_panel, panel);

	tianma->backlight->props.power = FB_BLANK_UNBLANK;
	backlight_update_status(tianma->backlight);

	return 0;
}

static int tianma_panel_disable(struct drm_panel *panel)
{
	struct tianma_panel *tianma =
		container_of(panel, struct tianma_panel, panel);

	tianma->backlight->props.power = FB_BLANK_POWERDOWN;
	backlight_update_status(tianma->backlight);

	return 0;
}

static const struct drm_panel_funcs tianma_panel_funcs = {
	.prepare = tianma_panel_prepare,
	.get_modes = tianma_panel_get_modes,
	.enable = tianma_panel_enable,
	.disable = tianma_panel_disable,
	.unprepare = tianma_panel_unprepare,
};

static struct backlight_ops tianma_bl_ops = {
	.update_status = tianma_bl_update_status,
	.get_brightness = tianma_bl_get_brightness,
};

static int tianma_panel_probe(struct mipi_dsi_device *dsi)
{
	struct tianma_panel *panel;
	int ret;
	struct backlight_properties bl_props = {
		.type = BACKLIGHT_RAW,
		.brightness = 0xfff,
		.max_brightness = 0xfff,
	};

	panel = devm_kzalloc(&dsi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;
	panel->dsi = dsi;

	mipi_dsi_set_drvdata(dsi, panel);

	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO
			  | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	ret = of_property_read_u32(dsi->dev.of_node, "dsi-lanes", &dsi->lanes);
	if (ret < 0) {
		dev_err(&dsi->dev, "Failed to get dsi-lanes property: %d", ret);
		return ret;
	}

	panel->vsp_vsn = devm_gpiod_get(&dsi->dev, "vsp-vsn", GPIOD_OUT_HIGH);
	panel->reset = devm_gpiod_get(&dsi->dev, "reset", GPIOD_OUT_LOW);

	panel->backlight = devm_backlight_device_register(
		&dsi->dev, dev_name(&dsi->dev), &dsi->dev, dsi, &tianma_bl_ops,
		&bl_props);
	if (IS_ERR(panel->backlight)) {
		ret = PTR_ERR(panel->backlight);
		dev_err(&dsi->dev, "Failed to register backlight: %d\n", ret);
		return ret;
	}

	drm_panel_init(&panel->panel);
	panel->panel.funcs = &tianma_panel_funcs;
	panel->panel.dev = &dsi->dev;

	ret = drm_panel_add(&panel->panel);
	if (ret < 0) {
		dev_err(&dsi->dev, "Failed to add panel: %d", ret);
		return ret;
	}

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(&dsi->dev, "Unable to mipi_dsi_attach! %d", ret);
		return ret;
	}

	return ret;
}

static int tianma_panel_remove(struct mipi_dsi_device *dsi)
{
	struct tianma_panel *panel = mipi_dsi_get_drvdata(dsi);
	gpiod_set_value(panel->vsp_vsn, 0);
	gpiod_set_value(panel->reset, 1);
	return 0;
}

static const struct of_device_id tianma_of_match[] = {
	{
		.compatible = "tianma,s6d6fp0a2",
	},
	{}};
MODULE_DEVICE_TABLE(of, tianma_of_match);

static struct mipi_dsi_driver tianma_panel_driver = {
	.driver =
		{
			.name = "panel-tianma-s6d6fp0a2",
			.of_match_table = tianma_of_match,
		},
	.probe = tianma_panel_probe,
	.remove = tianma_panel_remove,
};
module_mipi_dsi_driver(tianma_panel_driver);

MODULE_AUTHOR("Alex Van Damme <atv@google.com>");
MODULE_DESCRIPTION("Tianma Display panel");
MODULE_LICENSE("GPL v2");
