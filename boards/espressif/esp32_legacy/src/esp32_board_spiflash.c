/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_board_spiflash.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/spi/spi.h>
#ifdef CONFIG_ESP32_SPIFLASH_NXFFS
#include <nuttx/fs/nxffs.h>
#endif
#ifdef CONFIG_BCH
#include <nuttx/drivers/drivers.h>
#endif

// #include "esp32_spiflash.h"
#include "esp32_board_spiflash_setup.h"
#include "espressif/esp_spiflash.h"
#include "espressif/esp_spiflash_mtd.h"

// #define CONFIG_ESP32_SPIFLASH_SPIFFS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/****************************************************************************
 * Name: init_storage_partition
 *
 * Description:
 *   Initialize partition that is dedicated to general use.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#define CONFIG_ESP32_PARAM_MTD_OFFSET   0x330000
#define CONFIG_ESP32_PARAM_MTD_SIZE     0x10000

static int setup_spiffs(const char *path, struct mtd_dev_s *mtd,
                        const char *mnt_pt, int priv)
{
  int ret = OK;

  ret = register_mtddriver(path, mtd, priv, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register MTD: %d\n", ret);
      return -ENOMEM;
    }

  if (mnt_pt != NULL)
    {
      ret = nx_mount(path, mnt_pt, "spiffs", 0, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount the FS volume: %d\n", ret);
          return ret;
        }
    }

  return ret;
}

static int init_storage_partition(void)
{
	int ret = OK;
	struct mtd_dev_s *mtd;

	mtd = esp_spiflash_alloc_mtdpart(CONFIG_ESP32_PARAM_MTD_OFFSET,
					   CONFIG_ESP32_PARAM_MTD_SIZE);

	if (!mtd) {
		ferr("ERROR: Failed to alloc MTD partition of SPI Flash\n");
		return -ENOMEM;
	}

#ifdef CONFIG_ESP32_SPIFLASH_SMARTFS

	ret = setup_smartfs(0, mtd, NULL);

	if (ret < 0) {
		ferr("ERROR: Failed to setup smartfs\n");
		return ret;
	}

#elif defined(CONFIG_ESP32_SPIFLASH_NXFFS)

	ret = setup_nxffs(mtd, "/mnt");

	if (ret < 0) {
		ferr("ERROR: Failed to setup nxffs\n");
		return ret;
	}

#elif defined(CONFIG_ESP32_SPIFLASH_LITTLEFS)

	const char *path = "/dev/esp32flash";
	ret = setup_littlefs(path, mtd, NULL, 0755);

	if (ret < 0) {
		ferr("ERROR: Failed to setup littlefs\n");
		return ret;
	}

#elif defined(CONFIG_ESP32_SPIFLASH_SPIFFS)
	const char *path = "/dev/esp32flash";
	ret = setup_spiffs(path, mtd, "/data", 0755);

	if (ret < 0) {
		ferr("ERROR: Failed to setup spiffs\n");
		return ret;
	}

#else

	ret = register_mtddriver("/dev/esp32flash", mtd, 0755, NULL);

	if (ret < 0) {
		ferr("ERROR: Failed to register MTD: %d\n", ret);
		return ret;
	}

#endif

	return ret;
}


static int init_param_partition(void)
{
	int ret = OK;
	struct mtd_dev_s *mtd;

	mtd = esp_spiflash_alloc_mtdpart(CONFIG_ESP32_PARAM_MTD_OFFSET,
					   CONFIG_ESP32_PARAM_MTD_SIZE);

	if (!mtd) {
		ferr("ERROR: Failed to alloc PARAM MTD partition of SPI Flash\n");
		return -ENOMEM;
	}

	ret = register_mtddriver("/fs/mtd_params", mtd, 0755, NULL);

	if (ret < 0) {
		ferr("ERROR: Failed to register PARAM MTD: %d\n", ret);
		return ret;
	}

	return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_spiflash_init
 *
 * Description:
 *   Initialize the SPI Flash and register the MTD.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp32_spiflash_init(void)
{
	int ret = OK;


	ret = esp_spiflash_init();
	if (ret < 0) {
		return ret;
	}

	ret = init_storage_partition();
	if (ret < 0) {
		return ret;
	}

	// ret = init_param_partition();

	// if (ret < 0) {
	// 	return ret;
	// }


	return ret;
}
