################################################################################
#
# vl53l1x
#
################################################################################

VL53L1X_VERSION = 1.0
VL53L1X_SITE = $(BR2_EXTERNAL_VL53L1X_PATH)/package/vl53l1x/src
VL53L1X_SITE_METHOD = local
VL53L1X_LICENSE = GPL-2.0
VL53L1X_DEPENDENCIES = linux host-dtc

# Compile the device tree overlay and drop it where the Pi firmware
# looks for it, alongside the stock overlays.
define VL53L1X_INSTALL_DTBO
	mkdir -p $(BINARIES_DIR)/rpi-firmware/overlays
	$(HOST_DIR)/bin/dtc -@ -I dts -O dtb \
		-o $(BINARIES_DIR)/rpi-firmware/overlays/vl53l1x.dtbo \
		$(@D)/vl53l1x-overlay.dts
endef
VL53L1X_POST_INSTALL_TARGET_HOOKS += VL53L1X_INSTALL_DTBO

define VL53L1X_INSTALL_INIT_SYSV
	$(INSTALL) -D -m 0755 $(VL53L1X_PKGDIR)/S99vl53l1x \
		$(TARGET_DIR)/etc/init.d/S99vl53l1x
endef

$(eval $(kernel-module))
$(eval $(generic-package))
