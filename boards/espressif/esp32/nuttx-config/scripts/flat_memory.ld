/****************************************************************************
 * boards/xtensa/esp32/common/scripts/flat_memory.ld
 *
 * SPDX-License-Identifier: Apache-2.0
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
 * ESP32 Linker Script Memory Layout
 *
 * This file describes the memory layout (memory blocks) as virtual
 * memory addresses.
 *
 * <legacy/mcuboot>_sections.ld contains output sections to link compiler
 * output into these memory blocks.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include "esp32_aliases.ld"

#ifdef CONFIG_ESP32_FLASH_2M
#  define FLASH_SIZE 0x200000
#elif defined (CONFIG_ESP32_FLASH_4M)
#  define FLASH_SIZE 0x400000
#elif defined (CONFIG_ESP32_FLASH_8M)
#  define FLASH_SIZE 0x800000
#elif defined (CONFIG_ESP32_FLASH_16M)
#  define FLASH_SIZE 0x1000000
#endif

#define SRAM1_IRAM_LEN        0xa000
#define RTC_TIMER_RESERVE_RTC (24)
#define RESERVE_RTC_MEM       (RTC_TIMER_RESERVE_RTC)

MEMORY
{

  ROM (RX) :             org = 0x20,
                         len = FLASH_SIZE - ORIGIN(ROM)
  /* Below values assume the flash cache is on, and have the blocks this
   * uses subtracted from the length of the various regions. The 'data access
   * port' dram/drom regions map to the same iram/irom regions but are
   * connected to the data port of the CPU and e.g. allow bytewise access.
   */

  /* IRAM for PRO cpu. Not sure if happy with this, this is MMU area... */

  iram0_0_seg (RX) :     org = 0x40080000, len = 0x20000 + SRAM1_IRAM_LEN

  /* Flash mapped instruction data. */


  irom0_0_seg (RX) :     org = 0x400d0020, len = 0x330000 - 0x20

  /* Shared data RAM, excluding memory reserved for ROM bss/data/stack.
   * Enabling Bluetooth & Trace Memory features in menuconfig will decrease
   * the amount of RAM available.
   *
   * Note: The length of this section should be 0x50000, and this extra
   * DRAM is available in heap at runtime. However due to static ROM memory
   * usage at this 176KB mark, the additional static memory temporarily cannot
   * be used.
   */

  dram0_0_seg (RW) :     org = 0x3ffb0000 + CONFIG_ESP32_BT_RESERVE_DRAM,
                         len = 0x2c200 - CONFIG_ESP32_TRACEMEM_RESERVE_DRAM - CONFIG_ESP32_BT_RESERVE_DRAM

  /* Flash mapped constant data */

  drom0_0_seg (R) :      org = 0x3f400020, len = FLASH_SIZE - 0x20

  /* RTC fast memory (executable). Persists over deep sleep. */

  rtc_iram_seg (RWX) :   org = 0x400c0000, len = 0x2000

  /* RTC slow memory (data accessible). Persists over deep sleep.
   * Start of RTC slow memory is reserved for ULP co-processor code + data,
   * if enabled.
   */

  rtc_slow_seg (RW) :    org = 0x50000000 + CONFIG_ESP32_ULP_COPROC_RESERVE_MEM,
                         len = 0x1000 - CONFIG_ESP32_ULP_COPROC_RESERVE_MEM

  rtc_reserved_seg(RW) : org = 0x50000000 + 0x2000 - RESERVE_RTC_MEM, len = RESERVE_RTC_MEM

  /* External memory, including data and text */

  extmem_seg (RWX) :     org = 0x3f800000, len = 0x400000
}

  REGION_ALIAS("default_rodata_seg", drom0_0_seg);
  REGION_ALIAS("default_code_seg", irom0_0_seg);

_sram1_iram_start = 0x400a0000;
_sram1_iram_len = 0;

/* Heap ends at top of dram0_0_seg */

_eheap = 0x40000000 - 0;

_eiramheap = 0x400a0000;

/* Mark the end of the RTC heap (top of the RTC region) */

_ertcheap = 0x50001fff;
