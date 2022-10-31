// Copyright 2022 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Author: Yichao Zhang,  ETH Zurich
// Author: Samuel Riedel, ETH Zurich

#include <stdint.h>
#include <string.h>

#include "encoding.h"
#include "printf.h"
#include "runtime.h"
#include "synchronization.h"

#include "define.h"

// Define kernel include
#include "kernel/mat_mul.h"
#include "kernel/mat_mul_conflict_opt.h"

// Define memory distributing
int32_t matrix_a[matrix_M * matrix_N] __attribute__((section(".l1")));
int32_t matrix_b[matrix_N * matrix_P] __attribute__((section(".l1")));
int32_t matrix_c[matrix_M * matrix_P] __attribute__((section(".l1")));
#if defined(PARALLEL_CONCURRENT)
int32_t matrix_d[matrix_M * matrix_N] __attribute__((section(".l1")));
int32_t matrix_e[matrix_N * matrix_P] __attribute__((section(".l1")));
int32_t matrix_f[matrix_M * matrix_P] __attribute__((section(".l1")));
#endif
int volatile error __attribute__((section(".l2")));

// Function init_matrix
void init_matrix(int32_t *matrix, uint32_t num_rows, uint32_t num_columns,
                 int32_t a, int32_t b, int32_t c, uint32_t core_id,
                 uint32_t num_cores) {
  // How many rows/columns to split the matrix into
  uint32_t const split = 4;
  if (num_columns > num_rows) {
    // Parallelize over columns
    uint32_t const c_start = (num_rows / split) * (core_id % split);
    uint32_t const c_end = (num_rows / split) * ((core_id % split) + 1);
    for (uint32_t j = (core_id / split); j < num_columns;
         j += (num_cores / split)) {
      for (uint32_t i = c_start; i < c_end; ++i) {
        matrix[i * num_columns + j] = a * (int32_t)i + b * (int32_t)j + c;
      }
    }
  } else {
    // Parallelize over rows
    uint32_t const c_start = (num_columns / split) * (core_id % split);
    uint32_t const c_end = (num_columns / split) * ((core_id % split) + 1);
    for (uint32_t i = (core_id / split); i < num_rows;
         i += (num_cores / split)) {
      for (uint32_t j = c_start; j < c_end; ++j) {
        matrix[i * num_columns + j] = a * (int32_t)i + b * (int32_t)j + c;
      }
    }
  }
}

// Function verify_matrix
int verify_matrix(int32_t *matrix, uint32_t num_rows, uint32_t num_columns,
                  uint32_t inner_dim, int32_t aa, int32_t ab, int32_t ac,
                  int32_t ba, int32_t bb, int32_t bc, uint32_t core_id,
                  uint32_t num_cores) {
  // Convert to signed
  int32_t n = (int32_t)inner_dim;
  // Parallelize over rows
  for (uint32_t i = core_id; i < num_rows; i += num_cores) {
    for (uint32_t j = 0; j < num_columns; ++j) {
      int32_t ii = (int32_t)i;
      int32_t jj = (int32_t)j;
      int32_t lin =
          (aa * bb * ii * jj + aa * bc * ii + ac * bb * jj + ac * bc) * n;
      int32_t qua =
          ((aa * ba * ii + ab * bb * jj + ab * bc + ba * ac) * (n * (n - 1))) /
          2;
      int32_t cub = ((ab * ba) * (n * (n - 1) * (2 * n - 1))) / 6;
      int32_t golden = lin + qua + cub;
      if (matrix[i * num_columns + j] != golden) {
        return (i + j) == 0 ? -1 : (int)(i * num_columns + j);
      }
      matrix[i * num_columns + j] = 0;
    }
  }
  return 0;
}

// Function test_matrix_multiplication
#if defined(PARALLEL_CONCURRENT)
int test_matrix_multiplication(int32_t *__restrict__ A, int32_t *__restrict__ B,
                               int32_t *__restrict__ C, int32_t *__restrict__ D,
                               int32_t *__restrict__ E, int32_t *__restrict__ F,
                               uint32_t M, uint32_t N, uint32_t P,
                               uint32_t core_id, uint32_t num_cores) {
#else
int test_matrix_multiplication(int32_t *__restrict__ A, int32_t *__restrict__ B,
                               int32_t *__restrict__ C, uint32_t M, uint32_t N,
                               uint32_t P, uint32_t core_id,
                               uint32_t num_cores) {
#endif

  int32_t const A_a = 1;
  int32_t const A_b = 2;
  int32_t const A_c = -32;
  int32_t const B_a = 1;
  int32_t const B_b = 1;
  int32_t const B_c = 16;

  // Initialize Matrices
  init_matrix(A, M, N, A_a, A_b, A_c, core_id, num_cores);
  init_matrix(B, N, P, B_a, B_b, B_c, core_id, num_cores);
#if defined(PARALLEL_CONCURRENT)
  init_matrix(D, M, N, A_a, A_b, A_c, core_id, num_cores);
  init_matrix(E, N, P, B_a, B_b, B_c, core_id, num_cores);
#endif
  mempool_barrier(num_cores);

// Serial Benchmark
#if defined(SINGLE)
  if (core_id == 0) {
    printf("Serial Calculation Start\n");
    mempool_start_benchmark();
    mat_mul_unrolled_4x4_serial(A, B, C, M, N, P);
    mempool_stop_benchmark();
    printf("Calculation Finish\n");
  }
#endif

// Parallel Benchmark
#if defined(PARALLEL)
  if (core_id == 0) {
    printf("Parallel Calculation Start\n");
  }
  mempool_barrier(num_cores);

  if (core_id < NUM_PARALLEL_CORES) {
    mempool_start_benchmark();
    mat_mul_unrolled_4x4_conflict_opt_parallel_asm(A, B, C, M, N, P, core_id,
                                                   NUM_PARALLEL_CORES);
    mempool_start_benchmark();
    mempool_log_partial_barrier(2, core_id, NUM_PARALLEL_CORES);
    mempool_stop_benchmark();
  }
  mempool_barrier(num_cores);
#endif

// Concurrent Benchmark
#if defined(PARALLEL_CONCURRENT)
  if (core_id == 0) {
    printf("Concurrent Calculation Start\n");
  }
  mempool_barrier(num_cores);

  if (core_id < 512) {
    mempool_start_benchmark();
    mat_mul_unrolled_4x4_conflict_opt_parallel_asm(A, B, C, M, N, P, core_id,
                                                   512);
    mempool_start_benchmark();
    mempool_log_partial_barrier(2, core_id, 512);
    mempool_stop_benchmark();
  }
  if (core_id >= 512) {
    uint32_t core_id_new = core_id - 512;
    mempool_start_benchmark();
    mat_mul_unrolled_4x4_conflict_opt_parallel_asm(D, E, F, M, N, P,
                                                   core_id_new, 512);
    mempool_start_benchmark();
    mempool_log_partial_barrier(2, core_id, 512);
    mempool_stop_benchmark();
  }
  mempool_barrier(num_cores);
#endif

  // Verify results
  if (core_id == 0) {
    printf("Start Verify Results\n");
  }
  mempool_barrier(num_cores);
  if (verify_matrix(C, M, P, N, A_a, A_b, A_c, B_a, B_b, B_c, core_id,
                    num_cores)) {
    error = 1;
    return -1;
  }
  return 0;
}

// Main function block
int main() {
  uint32_t core_id = mempool_get_core_id();
  uint32_t num_cores = mempool_get_core_count();
  // Initialize barrier and synchronize
  mempool_barrier_init(core_id);

  if (core_id == 0) {
    error = 0;
  }

// Test the Matrix multiplication
#if defined(PARALLEL_CONCURRENT)
  test_matrix_multiplication(matrix_a, matrix_b, matrix_c, matrix_d, matrix_e,
                             matrix_f, matrix_M, matrix_N, matrix_P, core_id,
                             num_cores);
#else
  test_matrix_multiplication(matrix_a, matrix_b, matrix_c, matrix_M, matrix_N,
                             matrix_P, core_id, num_cores);
#endif
  // wait until all cores have finished
  mempool_barrier(num_cores);

  return error;
}