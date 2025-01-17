// Copyright 2021 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Author: Samuel Riedel, ETH Zurich
//         Matheus Cavalcante, ETH Zurich

#pragma once
#include "encoding.h"
#include <stddef.h>
#include <stdint.h>

extern char l1_alloc_base;
extern uint32_t atomic_barrier;
extern uint32_t wake_up_reg;

typedef uint32_t mempool_id_t;
typedef uint32_t mempool_timer_t;

/// Obtain the number of cores in the current cluster.
static inline mempool_id_t mempool_get_core_count() { return NUM_CORES; }

/// Obtain the ID of the current core.
static inline mempool_id_t mempool_get_core_id() {
  mempool_id_t r;
  asm volatile("csrr %0, mhartid" : "=r"(r));
  return r;
}

/// Reset a monotonically increasing cycle count.
static inline void mempool_start_benchmark() {
  asm volatile("" ::: "memory");
  write_csr(trace, 1);
  asm volatile("" ::: "memory");
}

/// Obtain a monotonically increasing cycle count.
static inline void mempool_stop_benchmark() {
  asm volatile("" ::: "memory");
  write_csr(trace, 0);
  asm volatile("" ::: "memory");
}

/// Obtain a monotonically increasing cycle count.
static inline mempool_timer_t mempool_get_timer() { return read_csr(mcycle); }

/// Busy loop for waiting
static inline void mempool_wait(uint32_t cycles) {
  asm volatile("1: \n\t"
               "addi %[counter], %[counter], -2 \n\t"
               "bgtz %[counter], 1b \n\t"
               : [counter] "+&r"(cycles)
               :
               : "memory");
}

static inline void mempool_wfi() { asm volatile("wfi"); }

// Wake up core with given core_id by writing in the wake up control register.
// If core_id equals -1, wake up all cores.
static inline void wake_up(uint32_t core_id) { wake_up_reg = core_id; }
static inline void wake_up_all() { wake_up((uint32_t)-1); }

// Dump a value via CSR
// This is only supported in simulation and an experimental feature. All writes
// to unimplemented CSR registers will be dumped by Snitch. This can be
// exploited to quickly print measurement values from all cores simultaneously
// without the hassle of printf. To specify multiple metrics, different CSRs can
// be used.
// The macro will define a function that will then always print via the same
// CSR. E.g., `dump(errors, 8)` will define a function with the following
// signature: `dump_errors(uint32_t val)`, which will print the given value via
// the 8th register.
// Alternatively, the `write_csr(reg, val)` macro can be used directly.
#define dump(name, reg)                                                        \
  static                                                                       \
      __attribute__((always_inline)) inline void dump_##name(uint32_t val) {   \
    asm volatile("csrw " #reg ", %0" ::"rK"(val));                             \
  }
