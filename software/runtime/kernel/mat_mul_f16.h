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

#include "xpulp/builtins_v2.h"

void matmul_2x2_single_f16_zfinx(__fp16 const *__restrict__ A,
                                 __fp16 const *__restrict__ B,
                                 __fp16 *__restrict__ C, uint32_t M, uint32_t N,
                                 uint32_t P) {
  for (uint32_t i = 0; i < M; i += 2) {
    for (uint32_t j = 0; j < P; j += 2) {
      __fp16 c00 = 0.0f;
      __fp16 c01 = 0.0f;
      __fp16 c10 = 0.0f;
      __fp16 c11 = 0.0f;
      for (uint32_t k = 0; k < N; k += 2) {
        // Explicitly load the values first to help with scheduling
        __fp16 val_a00 = A[(i + 0) * N + k + 0];
        __fp16 val_a01 = A[(i + 0) * N + k + 1];
        __fp16 val_a10 = A[(i + 1) * N + k + 0];
        __fp16 val_a11 = A[(i + 1) * N + k + 1];
        __fp16 val_b00 = B[(k + 0) * P + j + 0];
        __fp16 val_b01 = B[(k + 0) * P + j + 1];
        __fp16 val_b10 = B[(k + 1) * P + j + 0];
        __fp16 val_b11 = B[(k + 1) * P + j + 1];
        __fp16 mul00 = 0.0f;
        __fp16 mul01 = 0.0f;
        __fp16 mul10 = 0.0f;
        __fp16 mul11 = 0.0f;
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

void matmul_2x2_parallel_f16_zfinx(__fp16 const *__restrict__ A,
                                   __fp16 const *__restrict__ B,
                                   __fp16 *__restrict__ C, uint32_t M,
                                   uint32_t N, uint32_t P, uint32_t id,
                                   uint32_t numThreads) {
  // Parallelize by assigning each core one row
  uint32_t const c = 8; // How many columns to split the matrix into
  uint32_t const c_start = (P / c) * (id % c);
  uint32_t const c_end = (P / c) * ((id % c) + 1);
  for (uint32_t i = 2 * (id / c); i < M; i += 2 * (numThreads / c)) {
    for (uint32_t j = c_start; j < c_end; j += 2) {
      __fp16 c00 = 0.0f;
      __fp16 c01 = 0.0f;
      __fp16 c10 = 0.0f;
      __fp16 c11 = 0.0f;
      for (uint32_t k = 0; k < N; k += 2) {
        // Explicitly load the values first to help with scheduling
        __fp16 val_a00 = A[(i + 0) * N + k + 0];
        __fp16 val_a01 = A[(i + 0) * N + k + 1];
        __fp16 val_a10 = A[(i + 1) * N + k + 0];
        __fp16 val_a11 = A[(i + 1) * N + k + 1];
        __fp16 val_b00 = B[(k + 0) * P + j + 0];
        __fp16 val_b01 = B[(k + 0) * P + j + 1];
        __fp16 val_b10 = B[(k + 1) * P + j + 0];
        __fp16 val_b11 = B[(k + 1) * P + j + 1];
        __fp16 mul00 = 0.0f;
        __fp16 mul01 = 0.0f;
        __fp16 mul10 = 0.0f;
        __fp16 mul11 = 0.0f;
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

void matmul_4x2_parallel_f16_zfinx(const __fp16 *__restrict__ pSrcA,
                                      const __fp16 *__restrict__ pSrcB,
                                      __fp16 *__restrict__ pDstC,
                                      uint32_t M, uint32_t N,
                                      uint32_t P, uint32_t core_id,
                                      uint32_t numThreads) {

  uint32_t i = 0; // loop counter for M
  uint32_t j = 0; // loop counter for N
  uint32_t k = 0; // loop counter for P

  for (k = core_id; k < P / 2; k += numThreads) {
    for (i = 0; i < M / 4; i++) {
      __fp16 c00 = 0.0f;
      __fp16 c01 = 0.0f;
      __fp16 c10 = 0.0f;
      __fp16 c11 = 0.0f;
      __fp16 c20 = 0.0f;
      __fp16 c21 = 0.0f;
      __fp16 c30 = 0.0f;
      __fp16 c31 = 0.0f;
      for (j = 0; j < N / 2; j++) {

        __fp16 a00 = pSrcA[(i * 4) * N + (j * 2)];
        __fp16 a01 = pSrcA[(i * 4) * N + (j * 2) + 1];
        __fp16 a10 = pSrcA[(i * 4 + 1) * N + (j * 2)];
        __fp16 a11 = pSrcA[(i * 4 + 1) * N + (j * 2) + 1];
        __fp16 a20 = pSrcA[(i * 4 + 2) * N + (j * 2)];
        __fp16 a21 = pSrcA[(i * 4 + 2) * N + (j * 2) + 1];
        __fp16 a30 = pSrcA[(i * 4 + 3) * N + (j * 2)];
        __fp16 a31 = pSrcA[(i * 4 + 3) * N + (j * 2) + 1];

        __fp16 b00 = pSrcB[(j * 2) * P + (k * 2)];
        __fp16 b01 = pSrcB[(j * 2) * P + (k * 2) + 1];
        __fp16 b10 = pSrcB[(j * 2 + 1) * P + (k * 2)];
        __fp16 b11 = pSrcB[(j * 2 + 1) * P + (k * 2) + 1];

        asm volatile(

          "fmadd.h %[c00], %[a00], %[b00], %[c00];"
          "fmadd.h %[c00], %[a01], %[b10], %[c00];"
          "fmadd.h %[c01], %[a00], %[b01], %[c01];"
          "fmadd.h %[c01], %[a01], %[b11], %[c01];"
          "fmadd.h %[c10], %[a10], %[b00], %[c10];"
          "fmadd.h %[c10], %[a11], %[b10], %[c10];"
          "fmadd.h %[c11], %[a10], %[b01], %[c11];"
          "fmadd.h %[c11], %[a11], %[b11], %[c11];"
          "fmadd.h %[c20], %[a20], %[b00], %[c20];"
          "fmadd.h %[c20], %[a21], %[b10], %[c20];"
          "fmadd.h %[c21], %[a20], %[b01], %[c21];"
          "fmadd.h %[c21], %[a21], %[b11], %[c21];"
          "fmadd.h %[c30], %[a30], %[b00], %[c30];"
          "fmadd.h %[c30], %[a31], %[b10], %[c30];"
          "fmadd.h %[c31], %[a30], %[b01], %[c31];"
          "fmadd.h %[c31], %[a31], %[b11], %[c31];"

          : [c00] "=r" (c00), [c01] "=r" (c01), [c10] "=r" (c10), [c11] "=r" (c11),
            [c20] "=r" (c20), [c21] "=r" (c21), [c30] "=r" (c30), [c31] "=r" (c31)
          : [b00] "r" (b00), [b01] "r" (b01), [b10] "r" (b10), [b11] "r" (b11),
            [a00] "r" (a00), [a01] "r" (a01), [a10] "r" (a10), [a11] "r" (a11),
            [a20] "r" (a20), [a21] "r" (a21), [a30] "r" (a30), [a31] "r" (a31)
          :
        );
      }
      pDstC[(i * 4) * P + (k * 2)]         = c00;
      pDstC[(i * 4) * P + (k * 2 + 1)]     = c01;
      pDstC[(i * 4 + 1) * P + (k * 2)]     = c10;
      pDstC[(i * 4 + 1) * P + (k * 2 + 1)] = c11;
      pDstC[(i * 4 + 2) * P + (k * 2)]     = c20;
      pDstC[(i * 4 + 2) * P + (k * 2 + 1)] = c21;
      pDstC[(i * 4 + 3) * P + (k * 2)]     = c30;
      pDstC[(i * 4 + 3) * P + (k * 2 + 1)] = c31;

    }
  }
}

void matmul_4x2_parallel_f16vec_zfinx(const __fp16 *__restrict__ pSrcA,
                                      const __fp16 *__restrict__ pSrcB,
                                      __fp16 *__restrict__ pDstC,
                                      uint32_t M, uint32_t N,
                                      uint32_t P, uint32_t core_id,
                                      uint32_t numThreads) {

  uint32_t i = 0; // loop counter for M
  uint32_t j = 0; // loop counter for N
  uint32_t k = 0; // loop counter for P

  for (k = core_id; k < P / 2; k += numThreads) {
    for (i = 0; i < M / 4; i++) {
      float volatile sum00 = 0.0f;
      float volatile sum01 = 0.0f;
      float volatile sum10 = 0.0f;
      float volatile sum11 = 0.0f;
      float volatile sum20 = 0.0f;
      float volatile sum21 = 0.0f;
      float volatile sum30 = 0.0f;
      float volatile sum31 = 0.0f;
      for (j = 0; j < N / 2; j++) {

        v2h aVec0 = *(v2h*)&(pSrcA[(i * 4) * N + (j * 2)]);
        v2h aVec1 = *(v2h*)&(pSrcA[(i * 4 + 1) * N + (j * 2)]);
        v2h aVec2 = *(v2h*)&(pSrcA[(i * 4 + 2) * N + (j * 2)]);
        v2h aVec3 = *(v2h*)&(pSrcA[(i * 4 + 3) * N + (j * 2)]);
        v2h bVecTemp0 = *(v2h*)&(pSrcB[(j * 2) * P + (k * 2)]);
        v2h bVecTemp1 = *(v2h*)&(pSrcB[(j * 2 + 1) * P + (k * 2)]);
        unsigned bVec0 = 0U;
        unsigned bVec1 = 0U;
        unsigned TempH, TempL;

        asm volatile(
          "srli %[TempH], %[bVecTemp0], 0x10;"
          "slli %[TempH], %[TempH], 0x10;"
          "srli %[TempL], %[bVecTemp1], 0x10;"
          "or   %[bVec0], %[TempH], %[TempL];"

          "slli %[TempH], %[bVecTemp0], 0x10;"
          "slli %[TempL], %[bVecTemp1], 0x10;"
          "srli %[TempL], %[TempL], 0x10;"
          "or   %[bVec1], %[TempH], %[TempL];"

          "vfdotpex.s.h %[sum00], %[aVec0], %[bVec0];"
          "vfdotpex.s.h %[sum01], %[aVec0], %[bVec1];"
          "vfdotpex.s.h %[sum10], %[aVec1], %[bVec0];"
          "vfdotpex.s.h %[sum11], %[aVec1], %[bVec1];"
          "vfdotpex.s.h %[sum20], %[aVec2], %[bVec0];"
          "vfdotpex.s.h %[sum21], %[aVec2], %[bVec1];"
          "vfdotpex.s.h %[sum30], %[aVec3], %[bVec0];"
          "vfdotpex.s.h %[sum31], %[aVec3], %[bVec1];"

          : [sum00] "=r" (sum00), [sum01] "=r" (sum01), [sum10] "=r" (sum10), [sum11] "=r" (sum11),
            [sum20] "=r" (sum20), [sum21] "=r" (sum21), [sum30] "=r" (sum30), [sum31] "=r" (sum31),
            [bVec0] "=&r" (bVec0), [bVec1] "=&r" (bVec1), [TempH] "=&r" (TempH), [TempL] "=&r" (TempL)
          : [aVec0] "r" (aVec0), [aVec1] "r" (aVec1), [aVec2] "r" (aVec2), [aVec3] "r" (aVec3),
            [bVecTemp0] "r" (bVecTemp0), [bVecTemp1] "r" (bVecTemp1)
          :
        );
      }
      v2h res0, res1, res2, res3;
      asm volatile(
        "vfcpka.h.s %[res0], %[sum00], %[sum01];"
        "vfcpka.h.s %[res1], %[sum10], %[sum11];"
        "vfcpka.h.s %[res2], %[sum20], %[sum21];"
        "vfcpka.h.s %[res3], %[sum30], %[sum31];"
        : [res0] "=&r" (res0), [res1] "=&r" (res1), [res2] "=&r" (res2), [res3] "=&r" (res3)
        : [sum00] "r" (sum00), [sum01] "r" (sum01), [sum10] "r" (sum10), [sum11] "r" (sum11),
          [sum20] "r" (sum20), [sum21] "r" (sum21), [sum30] "r" (sum30), [sum31] "r" (sum31)
        :
      );
      ( *(v2h*)&pDstC[(i * 4) * P + (k * 2)]     ) = res0;
      ( *(v2h*)&pDstC[(i * 4 + 1) * P + (k * 2)] ) = res1;
      ( *(v2h*)&pDstC[(i * 4 + 2) * P + (k * 2)] ) = res2;
      ( *(v2h*)&pDstC[(i * 4 + 3) * P + (k * 2)] ) = res3;
    }
  }
}
