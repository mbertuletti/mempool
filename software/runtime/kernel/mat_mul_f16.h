// Copyright 2021 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Author: Marco Bertuletti, ETH Zurich

/* This library implements the matrix multiplication in multiple different ways.
 * The functions all follow the following format:
 *
 * A is an M x N matrix, B is a N x P matrix, and C is a M x P matrix
 * C = AB
 */

void matmul_2x2_single_f16_zfinx(float16 const *__restrict__ A,
                                 float16 const *__restrict__ B,
                                 float16 *__restrict__ C, uint32_t M, uint32_t N,
                                 uint32_t P) {

  for (uint32_t i = 0; i < M; i++) {
    for (uint32_t j = 0; j < P; j += 2) {
      float16 c00 = 0.0f;
      float16 c01 = 0.0f;
      float16 c10 = 0.0f;
      float16 c11 = 0.0f;
      for (uint32_t k = 0; k < N; k += 2) {
        // Explicitly load the values first to help with scheduling
        float16 val_a00 = A[(i + 0) * N + k + 0];
        float16 val_a01 = A[(i + 0) * N + k + 1];
        float16 val_a10 = A[(i + 1) * N + k + 0];
        float16 val_a11 = A[(i + 1) * N + k + 1];
        float16 val_b00 = B[(k + 0) * P + j + 0];
        float16 val_b01 = B[(k + 0) * P + j + 1];
        float16 val_b10 = B[(k + 1) * P + j + 0];
        float16 val_b11 = B[(k + 1) * P + j + 1];
        float16 mul00 = (float16) 0.0f;
        float16 mul01 = (float16) 0.0f;
        float16 mul10 = (float16) 0.0f;
        float16 mul11 = (float16) 0.0f;
        asm volatile(
            "fmadd.h %[c00], %[val_a00], %[val_b00], %[c00];"
            "fmadd.h %[c01], %[val_a00], %[val_b01], %[c01];"
            "fmadd.h %[c10], %[val_a10], %[val_b00], %[c10];"
            "fmadd.h %[c11], %[val_a10], %[val_b01], %[c11];"
            "fmadd.h %[c00], %[val_a01], %[val_b10], %[c00];"
            "fmadd.h %[c01], %[val_a01], %[val_b11], %[c01];"
            "fmadd.h %[c10], %[val_a11], %[val_b10], %[c10];"
            "fmadd.h %[c11], %[val_a11], %[val_b11], %[c11];"
            : [c00] "+&r"(c00), [c01] "+&r"(c01), [c10] "+&r"(c10),
              [c11] "+&r"(c11), [mul00] "+&r"(mul00), [mul01] "+&r"(mul01),
              [mul10] "+&r"(mul10), [mul11] "+&r"(mul11)
            : [val_a00] "r"(val_a00), [val_a01] "r"(val_a01),
              [val_a10] "r"(val_a10), [val_a11] "r"(val_a11),
              [val_b00] "r"(val_b00), [val_b01] "r"(val_b01),
              [val_b10] "r"(val_b10), [val_b11] "r"(val_b11));
        //        asm volatile(
        //          "fmul.h %[mul00], %[val_a00], %[val_b00];"
        //          "fmul.h %[mul01], %[val_a01], %[val_b10];"
        //          "fmul.h %[mul10], %[val_a00], %[val_b01];"
        //          "fmul.h %[mul11], %[val_a01], %[val_b11];"
        //          "fadd.h %[c00], %[c00], %[mul00];"
        //          "fadd.h %[c00], %[c00], %[mul01];"
        //          "fadd.h %[c01], %[c01], %[mul10];"
        //          "fadd.h %[c01], %[c01], %[mul11];"
        //          "fmul.h %[mul00], %[val_a10], %[val_b00];"
        //          "fmul.h %[mul01], %[val_a11], %[val_b10];"
        //          "fmul.h %[mul10], %[val_a10], %[val_b01];"
        //          "fmul.h %[mul11], %[val_a11], %[val_b11];"
        //          "fadd.h %[c10], %[c10], %[mul00];"
        //          "fadd.h %[c10], %[c10], %[mul01];"
        //          "fadd.h %[c11], %[c11], %[mul10];"
        //          "fadd.h %[c11], %[c11], %[mul11];"
        //          : [c00] "+&r"(c00), [c01] "+&r"(c01), [c10] "+&r"(c10),
        //          [c11] "+&r"(c11),
        //            [mul00] "+&r"(mul00), [mul01] "+&r"(mul01), [mul10]
        //            "+&r"(mul10), [mul11] "+&r"(mul11)
        //          : [val_a00] "r"(val_a00), [val_a01] "r"(val_a01), [val_a10]
        //          "r"(val_a10), [val_a11] "r"(val_a11),
        //            [val_b00] "r"(val_b00), [val_b01] "r"(val_b01), [val_b10]
        //            "r"(val_b10), [val_b11] "r"(val_b11));
      }
      C[(i + 0) * P + j + 0] = c00;
      C[(i + 0) * P + j + 1] = c01;
      C[(i + 1) * P + j + 0] = c10;
      C[(i + 1) * P + j + 1] = c11;
    }
  }
}

void matmul_2x2_parallel_f16_zfinx(float16 const *__restrict__ A,
                                   float16 const *__restrict__ B,
                                   float16 *__restrict__ C, uint32_t M,
                                   uint32_t N, uint32_t P, uint32_t id,
                                   uint32_t numThreads) {

  // Parallelize by assigning each core one row
  uint32_t const c = 8; // How many columns to split the matrix into
  uint32_t const c_start = (P / c) * (id % c);
  uint32_t const c_end = (P / c) * ((id % c) + 1);
  for (uint32_t i = 2 * (id / c); i < M; i += 2 * (numThreads / c)) {
    for (uint32_t j = c_start; j < c_end; j += 2) {
      float16 c00 = (float16) 0.0f;
      float16 c01 = (float16) 0.0f;
      float16 c10 = (float16) 0.0f;
      float16 c11 = (float16) 0.0f;
      for (uint32_t k = 0; k < N; k += 2) {
        // Explicitly load the values first to help with scheduling
        float16 val_a00 = A[(i + 0) * N + k + 0];
        float16 val_a01 = A[(i + 0) * N + k + 1];
        float16 val_a10 = A[(i + 1) * N + k + 0];
        float16 val_a11 = A[(i + 1) * N + k + 1];
        float16 val_b00 = B[(k + 0) * P + j + 0];
        float16 val_b01 = B[(k + 0) * P + j + 1];
        float16 val_b10 = B[(k + 1) * P + j + 0];
        float16 val_b11 = B[(k + 1) * P + j + 1];
        float16 mul00 = 0.0f;
        float16 mul01 = 0.0f;
        float16 mul10 = 0.0f;
        float16 mul11 = 0.0f;
        asm volatile(
            "fmadd.h %[c00], %[val_a00], %[val_b00], %[c00];"
            "fmadd.h %[c01], %[val_a00], %[val_b01], %[c01];"
            "fmadd.h %[c10], %[val_a10], %[val_b00], %[c10];"
            "fmadd.h %[c11], %[val_a10], %[val_b01], %[c11];"
            "fmadd.h %[c00], %[val_a01], %[val_b10], %[c00];"
            "fmadd.h %[c01], %[val_a01], %[val_b11], %[c01];"
            "fmadd.h %[c10], %[val_a11], %[val_b10], %[c10];"
            "fmadd.h %[c11], %[val_a11], %[val_b11], %[c11];"
            : [c00] "+&r"(c00), [c01] "+&r"(c01), [c10] "+&r"(c10),
              [c11] "+&r"(c11), [mul00] "+&r"(mul00), [mul01] "+&r"(mul01),
              [mul10] "+&r"(mul10), [mul11] "+&r"(mul11)
            : [val_a00] "r"(val_a00), [val_a01] "r"(val_a01),
              [val_a10] "r"(val_a10), [val_a11] "r"(val_a11),
              [val_b00] "r"(val_b00), [val_b01] "r"(val_b01),
              [val_b10] "r"(val_b10), [val_b11] "r"(val_b11));
        //        asm volatile(
        //          "fmul.h %[mul00], %[val_a00], %[val_b00];"
        //          "fmul.h %[mul01], %[val_a01], %[val_b10];"
        //          "fmul.h %[mul10], %[val_a00], %[val_b01];"
        //          "fmul.h %[mul11], %[val_a01], %[val_b11];"
        //          "fadd.h %[c00], %[c00], %[mul00];"
        //          "fadd.h %[c00], %[c00], %[mul01];"
        //          "fadd.h %[c01], %[c01], %[mul10];"
        //          "fadd.h %[c01], %[c01], %[mul11];"
        //          "fmul.h %[mul00], %[val_a10], %[val_b00];"
        //          "fmul.h %[mul01], %[val_a11], %[val_b10];"
        //          "fmul.h %[mul10], %[val_a10], %[val_b01];"
        //          "fmul.h %[mul11], %[val_a11], %[val_b11];"
        //          "fadd.h %[c10], %[c10], %[mul00];"
        //          "fadd.h %[c10], %[c10], %[mul01];"
        //          "fadd.h %[c11], %[c11], %[mul10];"
        //          "fadd.h %[c11], %[c11], %[mul11];"
        //          : [c00] "+&r"(c00), [c01] "+&r"(c01), [c10] "+&r"(c10),
        //          [c11] "+&r"(c11),
        //            [mul00] "+&r"(mul00), [mul01] "+&r"(mul01), [mul10]
        //            "+&r"(mul10), [mul11] "+&r"(mul11)
        //          : [val_a00] "r"(val_a00), [val_a01] "r"(val_a01), [val_a10]
        //          "r"(val_a10), [val_a11] "r"(val_a11),
        //            [val_b00] "r"(val_b00), [val_b01] "r"(val_b01), [val_b10]
        //            "r"(val_b10), [val_b11] "r"(val_b11));
      }
      C[(i + 0) * P + j + 0] = c00;
      C[(i + 0) * P + j + 1] = c01;
      C[(i + 1) * P + j + 0] = c10;
      C[(i + 1) * P + j + 1] = c11;
    }
  }
}

///*******************************/
///* ASM CODE KERNEL START BELOW */
///*******************************/

// Define immediate values that used in asm code.
#define N3  ((matrix_M - 3) * 4)
#define N31 ((-3 * matrix_N + 1) * 4)
#define P3  (matrix_P - 3) * 4
#define P31 ((-3 * matrix_N + 1) * 4)

dump(id, 1);

void matmul_4x4_parallel_f16_zfinx(float16 const *__restrict__ A,
                                   float16 const *__restrict__ B,
                                   float16 *__restrict__ C, uint32_t M,
                                   uint32_t N, uint32_t P, uint32_t id,
                                   uint32_t numThreads) {

#ifndef ASM

  /////////////////////////////
  //      Configuration      //
  /////////////////////////////
  // Parallelize by assigning each core one row
  // How many cores per window
  uint32_t c = numThreads / (M / 4);
  if (numThreads * 4 < M) {
    c = 1;
  }
  uint32_t const c_start = (P / c) * (id % c);
  uint32_t const c_end = (P / c) * ((id % c) + 1);

  // For avoiding group conflict by same tile
  // Each cores in the same tile should access to different groups
  uint32_t group_bank_nums = 256;           // MemPool = 256
  uint32_t tile_core_nums = 4;                 // MemPool = 4
  uint32_t jump_lines_A = group_bank_nums / N; // Used for i control
  uint32_t jump_lines_B = group_bank_nums / P; // Used for k control
  // Window size limit, min jump lines is 4 for MatrixA
  if (jump_lines_A < 4) {
    jump_lines_A = 4;
  }

  /////////////////////////////
  //      LOOP   OFFSET      //
  /////////////////////////////
  // Outer Loop Control, for group access port conflict
  uint32_t i_offset = jump_lines_A * (id % tile_core_nums);
  // Inner Loop Incremental Control, for group access port conflict
  uint32_t k_offset_incr = jump_lines_B * (id % tile_core_nums);
  // Inner Loop Control
  // k_offset = (Core offset) + (Window offset) + (Group offset from MatrixB)
  uint32_t k_offset = (id % c) + (2 * (id / c)) + k_offset_incr;
  // Middle Loop Control, window jump for avoiding bank conflict
  uint32_t conflict_row = (group_bank_nums * tile_core_nums) / P;
  uint32_t j_offset = (2 * (id / c)) / conflict_row;

  /////////////////////////////
  //      LOOP  CONTROL      //
  /////////////////////////////
  // Inner Round-Robin
  if (k_offset >= N) {
    k_offset = k_offset - N * (k_offset / N);
  }
  // Middle Round-Robin
  uint32_t window_in_P = (P / c) / 4;
  if (j_offset >= window_in_P) {
    j_offset = j_offset - window_in_P * (j_offset / window_in_P);
  }
  // Outer Loop Control
  uint32_t outer_loop_counter = 0;
  uint32_t outer_loop_time = M / (4 * numThreads);
  if (outer_loop_time < 1) {
    outer_loop_time = 1;
  }
  uint32_t M_partition = M / outer_loop_time;

  /////////////////////////////
  //      *LOOP  START*      //
  /////////////////////////////
  for (uint32_t i_ori = 4 * (id / c); i_ori < M; i_ori += 4 * (numThreads / c)) {
    outer_loop_counter += 1;
    uint32_t i = i_ori + i_offset;
    // Round-Robin control, if offset lines > M, back to the first window
    if (i >= M_partition * outer_loop_counter) {
      i = i - M_partition * (i / (M_partition * outer_loop_counter));
    }
    // Backup counter for mid-loop
    uint32_t j_offset_counter = c_start + j_offset * 4;
    uint32_t P_counter = c_end;

  Mid_loop:
    for (uint32_t j = j_offset_counter; j < P_counter; j += 4) {
      // Initialize 4x4 output tile
      float16 c00 = 0, c01 = 0, c02 = 0, c03 = 0;
      float16 c10 = 0, c11 = 0, c12 = 0, c13 = 0;
      float16 c20 = 0, c21 = 0, c22 = 0, c23 = 0;
      float16 c30 = 0, c31 = 0, c32 = 0, c33 = 0;

      // Backup the variables for restore and later use
      uint32_t k_offset_counter = k_offset;
      uint32_t N_counter = N;
      dump_id(j);

    Inner_Loop:
      for (uint32_t k = k_offset_counter; k < N_counter; k += 1) {
        // Explicitly load the values first to help with scheduling
        float16 b0 = B[k * P + j + 0];
        float16 b1 = B[k * P + j + 1];
        float16 b2 = B[k * P + j + 2];
        float16 b3 = B[k * P + j + 3];
        // A could be local with scrambling
        float16 a0 = A[(i + 0) * N + k];
        float16 a1 = A[(i + 1) * N + k];
        float16 a2 = A[(i + 2) * N + k];
        float16 a3 = A[(i + 3) * N + k];
        // Compute
        c00 += a0 * b0;
        c01 += a0 * b1;
        c02 += a0 * b2;
        c03 += a0 * b3;
        c10 += a1 * b0;
        c11 += a1 * b1;
        c12 += a1 * b2;
        c13 += a1 * b3;
        c20 += a2 * b0;
        c21 += a2 * b1;
        c22 += a2 * b2;
        c23 += a2 * b3;
        c30 += a3 * b0;
        c31 += a3 * b1;
        c32 += a3 * b2;
        c33 += a3 * b3;
        dump_id(k);
      }
      // Pseudo-jump code to avoid complie inner-loop twice
      // Complie twice will have scheduling issue due to register file limit.
      if (k_offset_counter > 0) {
        N_counter = k_offset;
        k_offset_counter = 0;
        goto Inner_Loop;
      }

      // Store
      C[(i + 0) * P + j + 0] = c00;
      C[(i + 0) * P + j + 1] = c01;
      C[(i + 0) * P + j + 2] = c02;
      C[(i + 0) * P + j + 3] = c03;
      C[(i + 1) * P + j + 0] = c10;
      C[(i + 1) * P + j + 1] = c11;
      C[(i + 1) * P + j + 2] = c12;
      C[(i + 1) * P + j + 3] = c13;
      C[(i + 2) * P + j + 0] = c20;
      C[(i + 2) * P + j + 1] = c21;
      C[(i + 2) * P + j + 2] = c22;
      C[(i + 2) * P + j + 3] = c23;
      C[(i + 3) * P + j + 0] = c30;
      C[(i + 3) * P + j + 1] = c31;
      C[(i + 3) * P + j + 2] = c32;
      C[(i + 3) * P + j + 3] = c33;
    }

    if (j_offset_counter != c_start) {
      P_counter = j_offset_counter;
      j_offset_counter = c_start;
      goto Mid_loop;
    }
  }

#else
  /////////////////////////////
  //      Configuration      //
  /////////////////////////////
  // Parallelize by assigning each core one row
  // How many cores per window
  uint32_t c = numThreads / (M / 4);
  if (numThreads * 4 < M) {
    c = 1;
  }
  uint32_t const c_start = (P / c) * (id % c);
  uint32_t const c_end = (P / c) * ((id % c) + 1);
  // For avoiding group conflict by same tile
  // Each cores in the same tile should access to different groups
  uint32_t group_bank_nums = 512;              // MemPool = 256
  uint32_t tile_core_nums = 8;                 // MemPool = 4
  uint32_t jump_lines_A = group_bank_nums / N; // Used for i control
  uint32_t jump_lines_B = group_bank_nums / P; // Used for k control
  // Window size limit, min jump lines is 4 for MatrixA
  if (jump_lines_A < 4) {
    jump_lines_A = 4;
  }
  /////////////////////////////
  //      LOOP   OFFSET      //
  /////////////////////////////
  // Outer Loop Control, for group access port conflict
  uint32_t i_offset = jump_lines_A * (id % tile_core_nums);
  // Inner Loop Incremental Control, for group access port conflict
  uint32_t k_offset_incr = jump_lines_B * (id % tile_core_nums);
  // Inner Loop Control
  // k_offset = (Core offset) + (Window offset) + (Group offset from MatrixB)
  uint32_t k_offset = (id % c) + (2 * (id / c)) + k_offset_incr;
  // Middle Loop Control, window jump for avoiding bank conflict
  uint32_t conflict_row = (group_bank_nums * tile_core_nums) / P;
  uint32_t j_offset = (2 * (id / c)) / conflict_row;
  /////////////////////////////
  //      LOOP  CONTROL      //
  /////////////////////////////
  // Inner Round-Robin
  if (k_offset >= N) {
    k_offset = k_offset - N * (k_offset / N);
  }
  // Middle Round-Robin
  uint32_t window_in_P = (P / c) / 4;
  if (j_offset >= window_in_P) {
    j_offset = j_offset - window_in_P * (j_offset / window_in_P);
  }
  // Outer Loop Control
  uint32_t outer_loop_counter = 0;
  uint32_t outer_loop_time = M / (4 * numThreads);
  if (outer_loop_time < 1) {
    outer_loop_time = 1;
  }
  uint32_t M_partition = M / outer_loop_time;

  /////////////////////////////
  //      *LOOP  START*      //
  /////////////////////////////
  for (uint32_t i_ori = 4 * (id / c); i_ori < M;
       i_ori += 4 * (numThreads / c)) {
    outer_loop_counter += 1;
    uint32_t i = i_ori + i_offset;
    // Round-Robin control, if offset lines > M, back to the first window
    if (i >= M_partition * outer_loop_counter) {
      i = i - M_partition * (i / (M_partition * outer_loop_counter));
    }
    // Backup counter for mid-loop
    uint32_t j_offset_counter = c_start + j_offset * 4;
    uint32_t P_counter = c_end;

  Mid_loop:
    for (uint32_t j = j_offset_counter; j < P_counter; j += 4) {
      // Address registers
      float16 const *addr_a_ori = &A[i * N];
      float16 const *addr_b_ori = &B[j];
      float16 const *addr_a = &A[i * N + k_offset];
      float16 const *addr_b = &B[k_offset * P + j];
      float16 const *end_b = &B[N * P + j];
      float16 const *addr_c = &C[i * P + j];
      register int32_t k asm("x1") = (int32_t)end_b;
      //      x12 x13 x14 x15
      //
      // x3   x16 x17 x18 x19
      // x4   x20 x21 x22 x23
      // x10  x24 x25 x26 x27
      // x11  x28 x29 x30 x31
      //
      //
      __asm__ volatile(
          ".balign 16 \n\t"
          // Outer loop: Initialize and preload. Execute this loop P times
          // TODO arrange
          "add sp, sp, -8 \n\t"
          "sw %[addr_b], 0(sp) \n\t"
          "sw %[addr_a_ori], 4(sp) \n\t"
          "p.lw  x3, %[N](%[addr_a]!) \n\t"
          "p.lw x12, 4(%[addr_b]!) \n\t"
          "p.lw x13, 4(%[addr_b]!) \n\t"
          "p.lw x14, 4(%[addr_b]!) \n\t"
          "p.lw x15, %[P_3](%[addr_b]!) \n\t" // Increment by P-3
          "p.lw  x4, %[N](%[addr_a]!) \n\t"
          "p.lw x10, %[N](%[addr_a]!) \n\t"
          "p.lw x11, %[N3_1](%[addr_a]!) \n\t" // Increment by -3N+1

          // If reach endpoint, swap address
          "bne %[addr_b], x1, init_comp \n\t"
          "lw x1, 0(sp) \n\t"
          "addi %[addr_a], %[addr_a_ori], 0 \n\t"
          "addi %[addr_b], %[addr_b_ori], 0 \n\t"
          "sw %[addr_b], 0(sp) \n\t"

          // Initial computation + prefetching
          "init_comp: \n\t"
          "fmul.h x16,  x3, x12 \n\t"
          "fmul.h x17,  x3, x13 \n\t"
          "fmul.h x18,  x3, x14 \n\t"
          "fmul.h x19,  x3, x15 \n\t"
          "p.lw  x3, %[N](%[addr_a]!) \n\t"
          "fmul.h x20,  x4, x12 \n\t"
          "fmul.h x21,  x4, x13 \n\t"
          "fmul.h x22,  x4, x14 \n\t"
          "fmul.h x23,  x4, x15 \n\t"
          "p.lw  x4, %[N](%[addr_a]!) \n\t"
          "fmul.h x24, x10, x12 \n\t"
          "fmul.h x25, x10, x13 \n\t"
          "fmul.h x26, x10, x14 \n\t"
          "fmul.h x27, x10, x15 \n\t"
          "p.lw x10, %[N](%[addr_a]!) \n\t"
          "fmul.h x28, x11, x12 \n\t"
          "p.lw x12, 4(%[addr_b]!) \n\t"
          "fmul.h x29, x11, x13 \n\t"
          "p.lw x13, 4(%[addr_b]!) \n\t"
          "fmul.h x30, x11, x14 \n\t"
          "p.lw x14, 4(%[addr_b]!) \n\t"
          "fmul.h %[addr_a_ori], x11, x15 \n\t" // Use addr_a_ori instead of x31
          "p.lw x15, %[P_3](%[addr_b]!) \n\t"   // Increment by P-3
          "p.lw x11, %[N3_1](%[addr_a]!) \n\t"  // Increment by -3N+1

          // If reach endpoint, swap address
          "bne %[addr_b], x1, inner_loop \n\t"
          "sw %[addr_a_ori], 8(sp) \n\t" // backup x31
          "lw %[addr_a_ori], 4(sp) \n\t" // load back addr_a_ori
          "lw x1, 0(sp) \n\t"
          "addi %[addr_a], %[addr_a_ori], 0 \n\t"
          "addi %[addr_b], %[addr_b_ori], 0 \n\t"
          "sw %[addr_b], 0(sp) \n\t"
          "lw %[addr_a_ori], 8(sp) \n\t" // load back x31

          // Inner loop: Do this loop N times
          "inner_loop: \n\t"
          "1: \n\t"
          "fmadd.h x16,  x3, x12, x16 \n\t"
          "fmadd.h x17,  x3, x13, x17 \n\t"
          "fmadd.h x20,  x4, x12, x20 \n\t"
          "fmadd.h x21,  x4, x13, x21 \n\t"
          "fmadd.h x18,  x3, x14, x18 \n\t"
          "fmadd.h x22,  x4, x14, x22 \n\t"
          "fmadd.h x19,  x3, x15, x19 \n\t"
          "p.lw  x3, %[N](%[addr_a]!) \n\t"
          "fmadd.h x23,  x4, x15, x23 \n\t"
          "p.lw  x4, %[N](%[addr_a]!) \n\t"
          "fmadd.h x24, x10, x12, x24 \n\t"
          "fmadd.h x28, x11, x12, x28 \n\t"
          "p.lw x12, 4(%[addr_b]!) \n\t"
          "fmadd.h x25, x10, x13, x25 \n\t"
          "fmadd.h x29, x11, x13, x29 \n\t"
          "p.lw x13, 4(%[addr_b]!) \n\t"
          "fmadd.h x26, x10, x14, x26 \n\t"
          "fmadd.h x30, x11, x14, x30 \n\t"
          "p.lw x14, 4(%[addr_b]!) \n\t"
          "fmadd.h x27, x10, x15, x27 \n\t"
          "fmadd.h %[addr_a_ori], x11, x15, %[addr_a_ori] \n\t"
          "p.lw x15, %[P_3](%[addr_b]!) \n\t" // Increment by P-3
          "p.lw x10, %[N](%[addr_a]!) \n\t"
          "p.lw x11, %[N3_1](%[addr_a]!) \n\t" // Increment by -3N+1
          "bne %[addr_b], x1, 1b \n\t"

          // Case1: Loop done if k_offset = 0
          // Case2: Loop done when 2nd time to here
          // Case3: If reach endpoint, swap address
          "lw %[addr_b], 0(sp) \n\t"
          "beq %[addr_b_ori], %[addr_b], store \n\t"
          "sw %[addr_a_ori], 8(sp) \n\t" // backup x31
          "lw %[addr_a_ori], 4(sp) \n\t" // load back addr_a_ori
          "addi x1, %[addr_b], 0 \n\t"
          "addi %[addr_a], %[addr_a_ori], 0 \n\t"
          "addi %[addr_b], %[addr_b_ori], 0 \n\t"
          "sw %[addr_b], 0(sp) \n\t"
          "lw %[addr_a_ori], 8(sp) \n\t" // load back x31
          "j 1b \n\t"

          // Loop done store
          "store: \n\t"
          "fmadd.h x16,  x3, x12, x16 \n\t"
          "fmadd.h x17,  x3, x13, x17 \n\t"
          "fmadd.h x18,  x3, x14, x18 \n\t"
          "p.sw x16, 4(%[addr_c]!) \n\t"
          "fmadd x19,  x3, x15, x19 \n\t"
          "p.sw x17, 4(%[addr_c]!) \n\t"
          "fmadd x20,  x4, x12, x20 \n\t"
          "p.sw x18, 4(%[addr_c]!) \n\t"
          "fmadd.h x21,  x4, x13, x21 \n\t"
          "p.sw x19, %[P_3](%[addr_c]!) \n\t"
          "fmadd.h x22,  x4, x14, x22 \n\t"
          "p.sw x20, 4(%[addr_c]!) \n\t"
          "fmadd.h x23,  x4, x15, x23 \n\t"
          "p.sw x21, 4(%[addr_c]!) \n\t"
          "fmadd.h x24, x10, x12, x24 \n\t"
          "p.sw x22, 4(%[addr_c]!) \n\t"
          "fmadd.h x25, x10, x13, x25 \n\t"
          "p.sw x23, %[P_3](%[addr_c]!) \n\t"
          "fmadd.h x26, x10, x14, x26 \n\t"
          "p.sw x24, 4(%[addr_c]!) \n\t"
          "fmadd.h x27, x10, x15, x27 \n\t"
          "p.sw x25, 4(%[addr_c]!) \n\t"
          "fmadd.h x28, x11, x12, x28 \n\t"
          "p.sw x26, 4(%[addr_c]!) \n\t"
          "fmadd.h x29, x11, x13, x29 \n\t"
          "p.sw x27, %[P_3](%[addr_c]!) \n\t"
          "fmadd.h x30, x11, x14, x30 \n\t"
          "p.sw x28, 4(%[addr_c]!) \n\t"
          "fmadd.h %[addr_a_ori], x11, x15, %[addr_a_ori] \n\t"
          "p.sw x29, 4(%[addr_c]!) \n\t"
          "p.sw x30, 4(%[addr_c]!) \n\t"
          "p.sw %[addr_a_ori], %[P_3](%[addr_c]!) \n\t"
          "add sp, sp, 8 \n\t"
          : [addr_a] "+&r"(addr_a), [addr_b] "+&r"(addr_b),
            [addr_c] "+&r"(addr_c), [addr_a_ori] "+&r"(addr_a_ori),
            [addr_b_ori] "+&r"(addr_b_ori) // Outputs
          : [N3_1] "r"(N31), [P_3] "I"(P3), [x1] "r"(k),
            [N] "I"(matrix_N * 4) // Inputs
          : "x3", "x4", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17",
            "x18", "x19", "x20", "x21", "x22", "x23", "x24", "x25", "x26",
            "x27", "x28", "x29", "x30", "memory"); // Cbber
    }
    if (j_offset_counter != c_start) {
      P_counter = j_offset_counter;
      j_offset_counter = c_start;
      goto Mid_loop;
    }
  }

#endif
}
