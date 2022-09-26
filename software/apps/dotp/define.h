// Copyright 2021 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Author: Marco Bertuletti, ETH Zurich

#define LEN       (1024)
#define N_PE      (16)
#define N_BANK    (NUM_CORES * 4)
#define N_BANK_PE (N_PE * 4)


//////////////////////////////////
/*          SELECT ONE          */

//#define SINGLE
//#define SINGLE_UNROLLED


#define PARALLEL
//#define PARALLEL_UNROLLED


//#define PARALLEL_RED0
//#define PARALLEL_UNROLLED_RED0
//#define STEP (256)
//#define STEP_CORES (STEP/4)

//#define PARALLEL_REDTREE
//#define PARALLEL_UNROLLED_REDTREE

//////////////////////////////////

// Vectors for kernel computation
int32_t vector_a[LEN] __attribute__((aligned(1024), section(".l1")));
int32_t vector_b[LEN] __attribute__((aligned(1024), section(".l1")));
//int32_t vector_a[LEN] __attribute__(( section(".l1")));
//int32_t vector_b[LEN] __attribute__(( section(".l1")));
#if defined(PARALLEL_RED0) || defined(PARALLEL_UNROLLED_RED0) || defined(PARALLEL_REDTREE) || defined(PARALLEL_UNROLLED_REDTREE)
int32_t sum[N_BANK] __attribute__((aligned(N_BANK), section(".l1")));
#else
int32_t sum         __attribute__((section(".l1")));
#endif



// Vectors for performance metrics
int32_t result   			__attribute__((section(".l1")));
int32_t check         __attribute__((section(".l1")));
int volatile error 		__attribute__((section(".l1")));
