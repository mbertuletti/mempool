// Copyright 2022 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Author: Marco Bertuletti, ETH Zurich

/* DEFINES */
#define N_CSAMPLES 64
#define N_RSAMPLES (2*N_CSAMPLES)
#define TEST_64

/* SINGLE */
#define SINGLE
// #define PRINT_SINGLE

/* PARALLEL */
// #define PARALLEL
// #define PRINT_PARALLEL

#define BIT_REV 1

/* DATA */
#define N_BANKS (1024)
#define N_TWIDDLES (3 * N_CSAMPLES / 4)

int16_t pSrc[8*N_BANKS] __attribute__((aligned(8*N_BANKS), section(".l1")));
int16_t pDst[8*N_BANKS] __attribute__((aligned(8*N_BANKS), section(".l1")));

#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define MAX(x, y) (((x) < (y)) ? (y) : (x))
