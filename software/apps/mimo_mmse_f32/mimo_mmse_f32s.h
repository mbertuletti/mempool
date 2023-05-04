// Copyright 2021 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Author: Marco Bertuletti, ETH Zurich

#pragma once
#define N_BANKS (NUM_CORES * 4)

void mempool_hermitian_f32s(float *pH, float *pG, float *sigma, const uint32_t n_rx, const uint32_t n_tx, const uint32_t folded);
void mempool_cholesky_f32s(float *pSrc, float *pL, const uint32_t n);
void mempool_Ltrisol_f32s(float *pL, float *in, float *x, const uint32_t n);
void mempool_Lttrisol_f32s(float *pL, float *in, float *x, const uint32_t n);

/* Computes the Hermitian matrix G = (H'*H + sigma^2I) */
void mempool_hermitian_f32s(float *pH, float *pG, float *sigma, const uint32_t n_rx, const uint32_t n_tx, const uint32_t folded) {

  uint32_t i, j, k;
  float a;
  float b;
  float c0, c1, c2, c3;
  float d0, d1, d2, d3;
  float as0, as1, as2, as3;
  float bs0, bs1, bs2, bs3;
  for (i = 0; i < n_tx; i++) {
    for (j = 0; j < n_tx; j += 4) {
      // Initialize the real part of sums
      as0 = 0.0f;
      as1 = 0.0f;
      as2 = 0.0f;
      as3 = 0.0f;
      // Initialize the imag part of sums
      bs0 = 0.0f;
      bs1 = 0.0f;
      bs2 = 0.0f;
      bs3 = 0.0f;
      // Inner Loop
      for (k = 0; k < n_rx; k++) {
        // inputs from matrix H_h
        a = pH[2U * (k * n_tx + i)];
        b = pH[2U * (k * n_tx + i) + 1U];
        // inputs from matrix H
        c0 = pH[2U * (k * n_tx + j)];
        c1 = pH[2U * (k * n_tx + j + 1U)];
        c2 = pH[2U * (k * n_tx + j + 2U)];
        c3 = pH[2U * (k * n_tx + j + 3U)];
        d0 = pH[2U * (k * n_tx + j) + 1U];
        d1 = pH[2U * (k * n_tx + j + 1U) + 1U];
        d2 = pH[2U * (k * n_tx + j + 2U) + 1U];
        d3 = pH[2U * (k * n_tx + j + 3U) + 1U];
        // dotproducts (ac + bd) + j (ad - bc)
        asm volatile (
          // a * c
          "fmadd.s  %[as0], %[a], %[c0], %[as0];"
          "fmadd.s  %[as1], %[a], %[c1], %[as1];"
          "fmadd.s  %[as2], %[a], %[c2], %[as2];"
          "fmadd.s  %[as3], %[a], %[c3], %[as3];"
          // a * d
          "fmadd.s  %[bs0], %[a], %[d0], %[bs0];"
          "fmadd.s  %[bs1], %[a], %[d1], %[bs1];"
          "fmadd.s  %[bs2], %[a], %[d2], %[bs2];"
          "fmadd.s  %[bs3], %[a], %[d3], %[bs3];"
          // b * d
          "fmadd.s  %[as0], %[b], %[d0], %[as0];"
          "fmadd.s  %[as1], %[b], %[d1], %[as1];"
          "fmadd.s  %[as2], %[b], %[d2], %[as2];"
          "fmadd.s  %[as3], %[b], %[d3], %[as3];"
          // - b * c
          "fnmsub.s %[bs0], %[b], %[c0], %[bs0];"
          "fnmsub.s %[bs1], %[b], %[c1], %[bs1];"
          "fnmsub.s %[bs2], %[b], %[c2], %[bs2];"
          "fnmsub.s %[bs3], %[b], %[c3], %[bs3];"
          : [as0] "+&r" (as0), [as1] "+&r" (as1), [as2] "+&r" (as2), [as3] "+&r" (as3),
            [bs0] "+&r" (bs0), [bs1] "+&r" (bs1), [bs2] "+&r" (bs2), [bs3] "+&r" (bs3)
          : [a] "r" (a), [b] "r" (b),
            [c0] "r" (c0), [c1] "r" (c1), [c2] "r" (c2), [c3] "r" (c3),
            [d0] "r" (d0), [d1] "r" (d1), [d2] "r" (d2), [d3] "r" (d3)
          :);
      }
      // Compute diagonal elements
      if (i == j) {
        asm volatile (
          "fadd.s  %[as0], %[as0], %[sigma];"
          : [as0] "+&r" (as0)
          : [sigma] "r" (sigma[i])
          :);
        bs0 = 0.0f;
      }
      else if (i == (j + 1U)) {
        asm volatile (
          "fadd.s  %[as1], %[as1], %[sigma];"
          : [as1] "+&r" (as1)
          : [sigma] "r" (sigma[i])
          :);
        bs1 = 0.0f;
      }
      else if (i == (j + 2U)) {
        asm volatile (
          "fadd.s  %[as2], %[as2], %[sigma];"
          : [as2] "+&r" (as2)
          : [sigma] "r" (sigma[i])
          :);
        bs2 = 0.0f;
      }
      else if (i == (j + 3U)) {
        asm volatile (
          "fadd.s  %[as3], %[as3], %[sigma];"
          : [as3] "+&r" (as3)
          : [sigma] "r" (sigma[i])
          :);
        bs3 = 0.0f;
      }
      if (!folded) {
        // Store
        pG[2 * (i * n_tx + j)] = as0;
        pG[2 * (i * n_tx + j + 1U)] = as1;
        pG[2 * (i * n_tx + j + 2U)] = as2;
        pG[2 * (i * n_tx + j + 3U)] = as3;
        pG[2 * (i * n_tx + j) + 1U] = bs0;
        pG[2 * (i * n_tx + j + 1U) + 1U] = bs1;
        pG[2 * (i * n_tx + j + 2U) + 1U] = bs2;
        pG[2 * (i * n_tx + j + 3U) + 1U] = bs3;
      } else {
        // Store
        uint32_t addr = i * ((n_tx / 2) * N_BANKS) + (j / 4) * (2 * N_BANKS);
        pG[addr] = as0;
        pG[addr + 1U] = bs0;
        pG[addr + 2U] = as1;
        pG[addr + 3U] = bs1;
        pG[addr + N_BANKS] = as2;
        pG[addr + N_BANKS + 1U] = bs2;
        pG[addr + N_BANKS + 2U] = as3;
        pG[addr + N_BANKS + 3U] = bs3;
      }
    }
  }
  return;
}

void mempool_MVP_conjtransp_f32s(float *pH, float *pb, float *py, const uint32_t n_rx, const uint32_t n_tx, const uint32_t folded) {

  uint32_t i, j;
  float a0, a1, a2, a3;
  float b0, b1, b2, b3;
  float c, d;
  float as0, as1, as2, as3;
  float bs0, bs1, bs2, bs3;

  i = 0;
  do {
    // Initialize the real part of sums
    as0 = 0.0f;
    as1 = 0.0f;
    as2 = 0.0f;
    as3 = 0.0f;
    // Initialize the imag part of sums
    bs0 = 0.0f;
    bs1 = 0.0f;
    bs2 = 0.0f;
    bs3 = 0.0f;
    for (j = 0; j < n_rx; j++) {
      // inputs from matrix H_h
      a0 = pH[2U * (j * n_tx + i)];
      a1 = pH[2U * (j * n_tx + i + 1U)];
      a2 = pH[2U * (j * n_tx + i + 2U)];
      a3 = pH[2U * (j * n_tx + i + 3U)];
      // inputs from matrix H_h
      b0 = pH[2U * (j * n_tx + i) + 1U];
      b1 = pH[2U * (j * n_tx + i + 1U) + 1U];
      b2 = pH[2U * (j * n_tx + i + 2U) + 1U];
      b3 = pH[2U * (j * n_tx + i + 3U) + 1U];
      // inputs from b
      c = pb[2U * j];
      d = pb[2U * j + 1U];
      asm volatile (
        // a * c
        "fmadd.s  %[as0], %[a0], %[c], %[as0];"
        "fmadd.s  %[as1], %[a1], %[c], %[as1];"
        "fmadd.s  %[as2], %[a2], %[c], %[as2];"
        "fmadd.s  %[as3], %[a3], %[c], %[as3];"
        // a * d
        "fmadd.s  %[bs0], %[a0], %[d], %[bs0];"
        "fmadd.s  %[bs1], %[a1], %[d], %[bs1];"
        "fmadd.s  %[bs2], %[a2], %[d], %[bs2];"
        "fmadd.s  %[bs3], %[a3], %[d], %[bs3];"
        // b * d
        "fmadd.s  %[as0], %[b0], %[d], %[as0];"
        "fmadd.s  %[as1], %[b1], %[d], %[as1];"
        "fmadd.s  %[as2], %[b2], %[d], %[as2];"
        "fmadd.s  %[as3], %[b3], %[d], %[as3];"
        // - b * c
        "fnmsub.s %[bs0], %[b0], %[c], %[bs0];"
        "fnmsub.s %[bs1], %[b1], %[c], %[bs1];"
        "fnmsub.s %[bs2], %[b2], %[c], %[bs2];"
        "fnmsub.s %[bs3], %[b3], %[c], %[bs3];"
        : [as0] "+&r" (as0), [as1] "+&r" (as1), [as2] "+&r" (as2), [as3] "+&r" (as3),
          [bs0] "+&r" (bs0), [bs1] "+&r" (bs1), [bs2] "+&r" (bs2), [bs3] "+&r" (bs3)
        : [c] "r" (c), [d] "r" (d),
          [a0] "r" (a0), [a1] "r" (a1), [a2] "r" (a2), [a3] "r" (a3),
          [b0] "r" (b0), [b1] "r" (b1), [b2] "r" (b2), [b3] "r" (b3)
        :);
    }
    if (!folded) {
      // Store
      py[2U * i] = as0;
      py[2U * (i + 1U)] = as1;
      py[2U * (i + 2U)] = as2;
      py[2U * (i + 3U)] = as3;
      py[2U * i + 1U] = bs0;
      py[2U * (i + 1U) + 1U] = bs1;
      py[2U * (i + 2U) + 1U] = bs2;
      py[2U * (i + 3U) + 1U] = bs3;
      i += 4;
    } else {
      // Store
      uint32_t addr = (i / 4) * 2 * N_BANKS;
      py[addr] = as0;
      py[addr + 1U] = bs0;
      py[addr + 2U] = as1;
      py[addr + 3U] = bs1;
      py[addr + N_BANKS] = as2;
      py[addr + N_BANKS + 1U] = bs2;
      py[addr + N_BANKS + 2U] = as3;
      py[addr + N_BANKS + 3U] = bs3;
      i += 4;
    }
  } while (i < n_tx);
  return;
}

void mempool_cholesky_f32s(float *pSrc, float *pL, const uint32_t n) {

  register float sum;
  float a, b;
  float c, d;
  float diag; // Diagonal element
  float ap, bp; // Pivot element
  float as, bs; // Sum element

  uint32_t i, j, k;

  for (j = 0; j < n; j++) {

      // Elements on diagonal (input matrix is positive-definite)
      ap = pSrc[2U * (j * n + j)];
      sum = 0.0f;
      for (k = 0; k < j; k++) {
        a = pL[2U * (j * n + k)];
        b = pL[2U * (j * n + k) + 1];
        asm volatile(
          "fmadd.s %[sum], %[a], %[a], %[sum];"
          "fmadd.s %[sum], %[b], %[b], %[sum];"
          : [sum] "+&r" (sum)
          : [a] "r" (a), [b] "r" (b) :);
      }
      asm volatile(
        "fsub.s %[ap], %[ap], %[sum];"
        "fsqrt.s  %[ap], %[ap];"
        : [ap] "+&r" (ap) : [sum] "r" (sum) :);
      pL[2U * (j * n + j)] = ap;

      // Elements on rows
      for (i = j + 1; i < n; i++) {
        // Pivot
        ap = pSrc[2U * (i * n + j)];
        bp = pSrc[2U * (i * n + j) + 1];
        // Diag
        diag = pL[2U * (j * n + j)];
        // Sum -> s = s + (ac + bd) + j*(bc - ad)
        as = 0.0f;
        bs = 0.0f;
        for (k = 0; k < j; k++) {
          a = pL[2U * (i * n + k)];
          b = pL[2U * (i * n + k) + 1];
          c = pL[2U * (j * n + k)];
          d = pL[2U * (j * n + k) + 1];
          asm volatile (
            "fmadd.s %[as], %[a], %[c], %[as];"
            "fmadd.s %[as], %[b], %[d], %[as];"
            "fmadd.s %[bs], %[b], %[c], %[bs];"
            "fnmsub.s %[bs], %[a], %[d], %[bs];"
            : [as] "+&r" (as), [bs] "+&r" (bs)
            : [a] "r" (a), [b] "r" (b), [c] "r" (c), [d] "r" (d)
            :);
        }
        asm volatile (
          "fsub.s %[ap], %[ap], %[as];"
          "fsub.s %[bp], %[bp], %[bs];"
          "fdiv.s %[ap], %[ap], %[diag];"
          "fdiv.s %[bp], %[bp], %[diag];"
          : [ap] "+&r" (ap), [bp] "+&r" (bp)
          : [diag] "r" (diag), [as] "r" (as), [bs] "r" (bs)
          :);
        pL[2U * (i * n + j)] = ap;
        pL[2U * (i * n + j) + 1] = bp;
      }

  }
  return;
}


void mempool_Ltrisol_f32s(float *pL, float *in, float *x, const uint32_t n) {

  uint32_t i, j;
  float a, b;
  float c, d;

  float as, bs;
  float ax, bx;
  float diag;

  // Solve for each variable x_i in turn
  for (i = 0; i < n; i++) {
    as = in[2U * i];
    bs = in[2U * i + 1];
    diag = pL[2 * (i * n + i)];
    // Use the previously solved variables to calculate the sum
    for (j = 0; j < i; j++) {
      a = pL[2U * (i * n + j)];
      b = pL[2U * (i * n + j) + 1];
      c = x[2U * j];
      d = x[2U * j + 1];
      asm volatile (
        "fnmsub.s %[as], %[a], %[c], %[as];"
        "fnmsub.s %[bs], %[a], %[d], %[bs];"
        "fmadd.s  %[as], %[b], %[d], %[as];"
        "fnmsub.s %[bs], %[b], %[c], %[bs];"
        : [as] "+&r" (as), [bs] "+&r" (bs)
        : [a] "r" (a), [b] "r" (b), [c] "r" (c), [d] "r" (d)
        :);
    }
    // Subtract the sum from b_i and divide by the diagonal element L[i][i]
    asm volatile (
      "fdiv.s %[ax], %[as], %[diag];"
      "fdiv.s %[bx], %[bs], %[diag];"
      : [ax] "+&r" (ax), [bx] "+&r" (bx)
      : [as] "r" (as), [bs] "r" (bs), [diag] "r" (diag)
      :);
    x[2U * i] = ax;
    x[2U * i + 1] = bx;
  }
  return;
}

void mempool_Lttrisol_f32s(float *pL, float *in, float *x, const uint32_t n) {

  uint32_t i, j;
  float a, b;
  float c, d;

  float as, bs;
  float ax, bx;
  float diag;

  // Solve for each variable x_i in turn
  for (i = 0; i < n; i++) {
    diag = pL[2 * ((n - 1 - i) * n + (n - 1 - i))];
    as = in[2 * (n - i - 1)];
    bs = in[2 * (n - i - 1) + 1];
    // Use the previously solved variables to calculate the sum
    for (j = 0; j < i; j++) {
      a = pL[2U * ((n - 1 - j) * n + (n - 1 - i))];
      b = pL[2U * ((n - 1 - j) * n + (n - 1 - i)) + 1];
      c = x[2U * (n - 1 - j)];
      d = x[2U * (n - 1 - j) + 1];
      asm volatile (
        "fnmsub.s  %[as], %[a], %[c], %[as];"
        "fnmsub.s %[as], %[b], %[d], %[as];"
        "fnmsub.s  %[bs], %[a], %[d], %[bs];"
        "fmadd.s  %[bs], %[b], %[c], %[bs];"
        : [as] "+&r" (as), [bs] "+&r" (bs)
        : [a] "r" (a), [b] "r" (b), [c] "r" (c), [d] "r" (d)
        :);
    }
    // Subtract the sum from b_i and divide by the diagonal element L[i][i]
    asm volatile (
      "fdiv.s %[ax], %[as], %[diag];"
      "fdiv.s %[bx], %[bs], %[diag];"
      : [ax] "+&r" (ax), [bx] "+&r" (bx)
      : [as] "r" (as), [bs] "r" (bs), [diag] "r" (diag)
      :);
    x[2U * (n - i - 1)] = ax;
    x[2U * (n - i - 1) + 1] = bx;
  }
  return;
}

void mempool_jacobi_f32s(float* pA, float* in, float* x, float tol, const uint32_t n, const uint32_t max_iter) {
  uint32_t i, j, k;
  float error;
  float register diff, den;
  float register as0, as1;
  float register bs0, bs1;
  float a0, a1;
  float b0, b1;
  float c0, c1;
  float d0, d1;
  for (k = 0; k < max_iter; k++) {

    // Initialize the diff variable
    diff = 0.0f;


    /* COMPUTE THE SUM */
    for (i = 0; i < n; i++) {

      den = pA[2U * (i * n + i)];
      as0 = in[2U * i];
      bs0 = in[2U * i + 1];
      as1 = 0.0f;
      bs1 = 0.0f;
      asm volatile (
        // divide the result by the pivot
        "fdiv.s    %[den], %[imm], %[den];"
        : [den] "+&r" (den) : [imm] "r" ((uint32_t)0x3F800000)
        :);

      /* COMPUTE OUTPUT */
      for(j = 0U; j < n; j += 2) {
        if (i == j) {
          a0 = pA[2U * (i * n + j + 1U)];
          b0 = pA[2U * (i * n + j + 1U) + 1U];
          c0 = x[2U * (j + 1U)];
          d0 = x[2U * (j + 1U) + 1U];
          // (ac - bd) + j * (ad + bc)
          asm volatile (
            "fnmsub.s  %[as0], %[a0], %[c0], %[as0];"
            "fnmsub.s  %[bs0], %[b0], %[c0], %[bs0];"
            "fmadd.s   %[as0], %[b0], %[d0], %[as0];"
            "fnmsub.s  %[bs0], %[a0], %[d0], %[bs0];"
            : [as0] "+&r" (as0), [bs0] "+&r" (bs0)
            : [a0] "r" (a0), [b0] "r" (b0), [c0] "r" (c0), [d0] "r" (d0)
            :);
        } else if(i == (j + 1U)) {
          a0 = pA[2U * (i * n + j)];
          b0 = pA[2U * (i * n + j) + 1U];
          c0 = x[2U * j];
          d0 = x[2U * j + 1U];
          // (ac - bd) + j * (ad + bc)
          asm volatile (
            "fnmsub.s  %[as0], %[a0], %[c0], %[as0];"
            "fnmsub.s  %[bs0], %[b0], %[c0], %[bs0];"
            "fmadd.s   %[as0], %[b0], %[d0], %[as0];"
            "fnmsub.s  %[bs0], %[a0], %[d0], %[bs0];"
            : [as0] "+&r" (as0), [bs0] "+&r" (bs0)
            : [a0] "r" (a0), [b0] "r" (b0), [c0] "r" (c0), [d0] "r" (d0)
            :);
        } else {
          a0 = pA[2U * (i * n + j)];
          a1 = pA[2U * (i * n + j + 1U)];
          b0 = pA[2U * (i * n + j) + 1U];
          b1 = pA[2U * (i * n + j + 1U) + 1U];
          c0 = x[2U * j];
          c1 = x[2U * (j + 1U)];
          d0 = x[2U * j + 1U];
          d1 = x[2U * (j + 1U) + 1U];
          // (ac - bd) + j * (ad + bc)
          asm volatile (
            "fnmsub.s  %[as0], %[a0], %[c0], %[as0];"
            "fnmsub.s  %[as1], %[a1], %[c1], %[as1];"
            "fnmsub.s  %[bs0], %[b0], %[c0], %[bs0];"
            "fnmsub.s  %[bs1], %[b1], %[c1], %[bs1];"
            "fmadd.s   %[as0], %[b0], %[d0], %[as0];"
            "fmadd.s   %[as1], %[b1], %[d1], %[as1];"
            "fnmsub.s  %[bs0], %[a0], %[d0], %[bs0];"
            "fnmsub.s  %[bs1], %[a1], %[d1], %[bs1];"
            : [as0] "+&r" (as0), [as1] "+&r" (as1),
              [bs0] "+&r" (bs0), [bs1] "+&r" (bs1)
            : [a0] "r" (a0), [a1] "r" (a1),
              [b0] "r" (b0), [b1] "r" (b1),
              [c0] "r" (c0), [c1] "r" (c1),
              [d0] "r" (d0), [d1] "r" (d1)
            :);
        }
      }
      // Partial sums
      asm volatile (
        "fadd.s %[as0], %[as1], %[as0];"
        "fadd.s %[bs0], %[bs1], %[bs0];"
        : [as0] "+&r" (as0), [bs0] "+&r" (bs0)
        : [as1] "r" (as1), [bs1] "r" (bs1)
        :);

      /* COMPUTE THE NEW DATA (& DIFFERENCE)*/
      asm volatile (
        // divide the result by the pivot & compute difference
        "fmul.s    %[as0], %[as0], %[den];"
        "fmul.s    %[bs0], %[bs0], %[den];"
        : [as0] "+&r" (as0), [bs0] "+&r" (bs0)
        : [den] "r" (den)
        :);
      // Load the previous result
      a0 = x[2U * i];
      b0 = x[2U * i + 1];
      asm volatile (
        "fsub.s    %[a0], %[a0], %[as0];"
        "fsub.s    %[b0], %[b0], %[bs0];"
      : [a0] "+&r" (a0), [b0] "+&r" (b0)
      : [as0] "r" (as0), [bs0] "r" (bs0)
      :);
      // Add absolute value to total difference
      a0 = (a0 > 0.0f) ? a0 : (-a0);
      b0 = (b0 > 0.0f) ? b0 : (-b0);
      asm volatile (
        "fadd.s %[diff], %[a0], %[diff];"
        "fadd.s %[diff], %[b0], %[diff];"
        : [diff] "+&r" (diff)
        : [a0] "r" (a0), [b0] "r" (b0)
        :);

      /* STORE THE RESULT */
      x[2U * i] = as0;
      x[2U * i + 1U] = bs0;
    }

    /* COMPUTE THE ERROR */
    asm volatile (
      // divide the result by the pivot
      "fdiv.s    %[error], %[diff], %[n_samples];"
      : [error] "+&r" (error)
      : [diff] "r" (diff), [n_samples] "r" (2.0f * (float)n)
      :);
    if (error < tol)
      break;

  }
  return;
}

void mempool_cholesky_folded_f32s(float *pSrc, float *pL, const uint32_t n) {

  register float sum;

  // first matrix row:
  //           item[0-2] item[1-3]
  // memrow[0]    x x       x x
  // memrow[1]    x x       x x
  // second matrix row:
  // memrow[2]    x x       x x
  // memrow[3]    x x       x x
  // third matrix row:
  // memrow[4]    x x       x x
  // memrow[5]    x x       x x
  // i * memrow_xrow * N_BANKS + (j / local_items) * N_BANKS + j % local_items

  float a, b;
  float c, d;
  float diag; // Diagonal element
  float ap, bp; // Pivot element
  float as, bs; // Sum element
  uint32_t banks_per_row = (n / 2) * N_BANKS;

  uint32_t i, j, k;
  for (j = 0; j < n; j++) {
      // Elements on diagonal (input matrix is positive-definite)
      ap = pSrc[j * banks_per_row + (j / 2) * N_BANKS + 2 * (j % 2)];
      sum = 0.0f;
      for (k = 0; k < j; k++) {
        a = pL[j * banks_per_row + (k / 2) * N_BANKS + 2 * (k % 2)];
        b = pL[j * banks_per_row + (k / 2) * N_BANKS + 2 * (k % 2) + 1];
        asm volatile(
          "fmadd.s %[sum], %[a], %[a], %[sum];"
          "fmadd.s %[sum], %[b], %[b], %[sum];"
          : [sum] "+&r" (sum)
          : [a] "r" (a), [b] "r" (b) :);
      }
      asm volatile(
        "fsub.s %[ap], %[ap], %[sum];"
        "fsqrt.s  %[ap], %[ap];"
        : [ap] "+&r" (ap) : [sum] "r" (sum) :);
      pL[ j * banks_per_row + (j / 2) * N_BANKS + 2 * (j % 2) ] = ap;
      // Elements on rows
      for (i = j + 1; i < n; i++) {
        // Pivot
        ap = pSrc[i * banks_per_row + (j / 2) * N_BANKS + 2 * (j % 2)];
        bp = pSrc[i * banks_per_row + (j / 2) * N_BANKS + 2 * (j % 2) + 1];
        // Diag
        diag = pL[j * banks_per_row + (j / 2) * N_BANKS + 2 * (j % 2)];
        // Sum -> s = s + (ac + bd) + j*(bc - ad)
        as = 0.0f;
        bs = 0.0f;
        for (k = 0; k < j; k++) {
          a = pL[i * banks_per_row + (k / 2) * N_BANKS + 2 * (k % 2)];
          b = pL[i * banks_per_row + (k / 2) * N_BANKS + 2 * (k % 2) + 1];
          c = pL[j * banks_per_row + (k / 2) * N_BANKS + 2 * (k % 2)];
          d = pL[j * banks_per_row + (k / 2) * N_BANKS + 2 * (k % 2) + 1];
          asm volatile (
            "fmadd.s %[as], %[a], %[c], %[as];"
            "fmadd.s %[as], %[b], %[d], %[as];"
            "fmadd.s %[bs], %[b], %[c], %[bs];"
            "fnmsub.s %[bs], %[a], %[d], %[bs];"
            : [as] "+&r" (as), [bs] "+&r" (bs)
            : [a] "r" (a), [b] "r" (b), [c] "r" (c), [d] "r" (d)
            :);
        }
        asm volatile (
          "fsub.s %[ap], %[ap], %[as];"
          "fsub.s %[bp], %[bp], %[bs];"
          "fdiv.s %[ap], %[ap], %[diag];"
          "fdiv.s %[bp], %[bp], %[diag];"
          : [ap] "+&r" (ap), [bp] "+&r" (bp)
          : [diag] "r" (diag), [as] "r" (as), [bs] "r" (bs)
          :);
        pL[i * banks_per_row + (j / 2) * N_BANKS + 2 * (j % 2)] = ap;
        pL[i * banks_per_row + (j / 2) * N_BANKS + 2 * (j % 2) + 1] = bp;
      }
  }
  return;
}


void mempool_Ltrisol_folded_f32s(float *pL, float *in, float *x, const uint32_t n) {

  uint32_t i, j;
  float a, b;
  float c, d;

  float as, bs;
  float ax, bx;
  float diag;
  uint32_t banks_per_row = (n / 2) * N_BANKS;

  // Solve for each variable x_i in turn
  for (i = 0; i < n; i++) {
    diag = pL[i * banks_per_row + (i / 2) * N_BANKS + 2 * (i % 2)];
    as = in[(i / 2) * N_BANKS + 2 * (i % 2)];
    bs = in[(i / 2) * N_BANKS + 2 * (i % 2) + 1];
    // Use the previously solved variables to calculate the sum
    for (j = 0; j < i; j++) {
      a = pL[i * banks_per_row + (j / 2) * N_BANKS + 2 * (j % 2)];
      b = pL[i * banks_per_row + (j / 2) * N_BANKS + 2 * (j % 2) + 1];
      c = x[(j / 2) * N_BANKS + 2 * (j % 2)];
      d = x[(j / 2) * N_BANKS + 2 * (j % 2) + 1];
      asm volatile (
        "fnmsub.s %[as], %[a], %[c], %[as];"
        "fnmsub.s %[bs], %[a], %[d], %[bs];"
        "fmadd.s  %[as], %[b], %[d], %[as];"
        "fnmsub.s %[bs], %[b], %[c], %[bs];"
        : [as] "+&r" (as), [bs] "+&r" (bs)
        : [a] "r" (a), [b] "r" (b), [c] "r" (c), [d] "r" (d)
        :);
    }
    // Subtract the sum from b_i and divide by the diagonal element L[i][i]
    asm volatile (
      "fdiv.s %[ax], %[as], %[diag];"
      "fdiv.s %[bx], %[bs], %[diag];"
      : [ax] "+&r" (ax), [bx] "+&r" (bx)
      : [as] "r" (as), [bs] "r" (bs), [diag] "r" (diag)
      :);
    x[(i / 2) * N_BANKS + 2 * (i % 2)] = ax;
    x[(i / 2) * N_BANKS + 2 * (i % 2) + 1] = bx;
  }
  return;
}

void mempool_Lttrisol_folded_f32s(float *pL, float *in, float *x, const uint32_t n) {

  uint32_t i, j;
  float a, b;
  float c, d;

  float as, bs;
  float ax, bx;
  float diag;
  uint32_t banks_per_row = (n / 2) * N_BANKS;

  // Solve for each variable x_i in turn
  for (i = 0; i < n; i++) {
    // reversed i index
    uint32_t rev_i = (n - 1 - i);
    diag = pL[rev_i * banks_per_row + (rev_i / 2) * N_BANKS + 2 * (rev_i % 2)];
    as = in[(rev_i / 2) * N_BANKS + 2 * (rev_i % 2)];
    bs = in[(rev_i / 2) * N_BANKS + 2 * (rev_i % 2) + 1];
    // Use the previously solved variables to calculate the sum
    for (j = 0; j < i; j++) {
      // reversed j index
      uint32_t rev_j = (n - 1 - j);
      a = pL[rev_j * banks_per_row + (rev_i / 2) * N_BANKS + 2 * (rev_i % 2)];
      b = pL[rev_j * banks_per_row + (rev_i / 2) * N_BANKS + 2 * (rev_i % 2) + 1];
      c = x[2U * rev_j];
      d = x[2U * rev_j + 1];
      asm volatile (
        "fnmsub.s  %[as], %[a], %[c], %[as];"
        "fnmsub.s %[as], %[b], %[d], %[as];"
        "fnmsub.s  %[bs], %[a], %[d], %[bs];"
        "fmadd.s  %[bs], %[b], %[c], %[bs];"
        : [as] "+&r" (as), [bs] "+&r" (bs)
        : [a] "r" (a), [b] "r" (b), [c] "r" (c), [d] "r" (d)
        :);
    }
    // Subtract the sum from b_i and divide by the diagonal element L[i][i]
    asm volatile (
      "fdiv.s %[ax], %[as], %[diag];"
      "fdiv.s %[bx], %[bs], %[diag];"
      : [ax] "+&r" (ax), [bx] "+&r" (bx)
      : [as] "r" (as), [bs] "r" (bs), [diag] "r" (diag)
      :);
    x[2U * rev_i] = ax;
    x[2U * rev_i + 1] = bx;
  }
  return;
}
