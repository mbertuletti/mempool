// Copyright 2021 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

/* This file will get processed by the precompiler to expand all macros. */

MEMORY {
  l1 (R) : ORIGIN = 0x00000000, LENGTH = (NUM_CORES * 0x1000) /* NUM_CORES * 4 * 1KiB per bank */
  l2     : ORIGIN = L2_BASE   , LENGTH = L2_SIZE
  rom (R): ORIGIN = BOOT_ADDR , LENGTH = 0x00001000
}

SECTIONS {
  // Start end end of memories
  __l1_start = ORIGIN(l1);
  __l1_end = ORIGIN(l1) + LENGTH(l1);
  __l2_start = ORIGIN(l2);
  __l2_end = ORIGIN(l2) + LENGTH(l2);
  __rom_start = ORIGIN(rom);
  __rom_end = ORIGIN(rom) + LENGTH(rom);

  // Stack size
  __stack_start = __l1_start;
  __stack_end = __l1_start + (NUM_CORES * 0x400);

  // Hardware register location
  eoc_reg                = 0x40000000;
  wake_up_reg            = 0x40000004;
  tcdm_start_address_reg = 0x40000008;
  tcdm_end_address_reg   = 0x4000000C;
  nr_cores_address_reg   = 0x40000010;
  ro_cache_enable        = 0x40000014;
  ro_cache_flush         = 0x40000018;
  ro_cache_start_0       = 0x4000001C;
  ro_cache_end_0         = 0x40000020;
  ro_cache_start_1       = 0x40000024;
  ro_cache_end_1         = 0x40000028;
  ro_cache_start_2       = 0x4000002C;
  ro_cache_end_2         = 0x40000030;
  ro_cache_start_3       = 0x40000034;
  ro_cache_end_3         = 0x40000038;

  fake_uart              = 0xC0000000;
}
