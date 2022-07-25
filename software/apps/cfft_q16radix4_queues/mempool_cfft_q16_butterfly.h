// Copyright 2022 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Author: Marco Bertuletti, ETH Zurich


static void mempool_radix4_butterfly_q16s_xpulpimg(  int16_t *pSrc16,
                                            uint32_t fftLen,
                                            int16_t *pCoef16,
                                            uint32_t twidCoefModifier);

static void mempool_radix4_butterfly_q16p_xpulpimg(  int16_t *pSrc16,
                                            uint32_t fftLen,
                                            const int16_t *pCoef16,
                                            uint32_t twidCoefModifier,
                                            uint32_t nPE);

static inline void radix4_butterfly_first( int16_t* pSrc16,
                                            uint32_t i0,
                                            uint32_t n2,
                                            v2s CoSi1,
                                            v2s CoSi2,
                                            v2s CoSi3,
                                            v2s C1,
                                            v2s C2,
                                            v2s C3);

static inline void radix4_butterfly_middle(  int16_t* pSrc16,
                                              uint32_t i0,
                                              uint32_t n2,
                                              v2s CoSi1,
                                              v2s CoSi2,
                                              v2s CoSi3,
                                              v2s C1,
                                              v2s C2,
                                              v2s C3);

static inline void radix4_butterfly_last(  int16_t* pSrc16,
                                            uint32_t i0,
                                            uint32_t n2);


static void mempool_radix4_butterfly_q16s_xpulpimg( int16_t *pSrc16,
                                                    uint32_t fftLen,
                                                    int16_t *pCoef16,
                                                    uint32_t twidCoefModifier) {

    v2s CoSi1, CoSi2, CoSi3;
    v2s C1, C2, C3;
    v2s A, B, C, D, E, F, G, H;
    int16_t t0, t1, t2, t3, t4, t5;
    uint32_t i0, i1, i2, i3;
    uint32_t n1, n2, ic, j, k;

    queue_t *data0_queue = 0;
    queue_t *data1_queue = 0;
    queue_t *data2_queue = 0;
    queue_t *data3_queue = 0;

    /* FIRST STAGE */

    n2 = fftLen;
    n1 = n2;
    n2 >>= 2U;

    //queue_domain_create(get_alloc_tile(0), &data0_queue, n2);
    //queue_domain_create(get_alloc_tile(1), &data1_queue, n2);
    //queue_domain_create(get_alloc_tile(2), &data2_queue, n2);
    //queue_domain_create(get_alloc_tile(3), &data3_queue, n2);
    //queue_create(&data0_queue, n2);
    //queue_create(&data1_queue, n2);
    //queue_create(&data2_queue, n2);
    //queue_create(&data3_queue, n2);

    for(i0 = 0; i0 < n2; i0++) {
        i1 = i0 + n2;
        i2 = i1 + n2;
        i3 = i2 + n2;
        blocking_queue_push(data0_queue, (int32_t*) &pSrc16[i0 * 2U]);
        blocking_queue_push(data1_queue, (int32_t*) &pSrc16[i1 * 2U]);
        blocking_queue_push(data2_queue, (int32_t*) &pSrc16[i2 * 2U]);
        blocking_queue_push(data3_queue, (int32_t*) &pSrc16[i3 * 2U]);
    }

    /*Index for twiddle coefficient */
    ic = 0U;
    for(i0 = 0; i0 < n2; i0++) {

        /* co1 & si1 are read from Coefficient pointer */
        CoSi1 = *(v2s *)&pCoef16[ic * 2U];
        /* co2 & si2 are read from Coefficient pointer */
        CoSi2 = *(v2s *)&pCoef16[2U * ic * 2U];
        /* co3 & si3 are read from Coefficient pointer */
        CoSi3 = *(v2s *)&pCoef16[3U * (ic * 2U)];
        t0 = (int16_t) CoSi1[0];
        t1 = (int16_t) CoSi1[1];
        t2 = (int16_t) CoSi2[0];
        t3 = (int16_t) CoSi2[1];
        t4 = (int16_t) CoSi3[0];
        t5 = (int16_t) CoSi3[1];
        C1 = __PACK2(-t1, t0);
        C2 = __PACK2(-t3, t2);
        C3 = __PACK2(-t5, t4);

        int32_t read_data0;
        int32_t read_data1;
        int32_t read_data2;
        int32_t read_data3;
        blocking_queue_pop(data0_queue, &read_data0);
        blocking_queue_pop(data1_queue, &read_data1);
        blocking_queue_pop(data2_queue, &read_data2);
        blocking_queue_pop(data3_queue, &read_data3);

        B = __SRA2((v2s)read_data1, ((v2s){ 2, 2 }));
        /* Read yd (real), xd(imag) input */
        D = __SRA2((v2s)read_data3, ((v2s){ 2, 2 }));
        /* Read ya (real), xa (imag) input */
        A = __SRA2((v2s)read_data0, ((v2s){ 2, 2 }));
        /* Read yc (real), xc(imag) input */
        C = __SRA2((v2s)read_data2, ((v2s){ 2, 2 }));
        /* G0 = (yb + yd), G1 = (xb + xd) */
        G = __ADD2(B, D);
        /* H0 = (yb - yd), H1 = (xb - xd) */
        H = __SUB2(B, D);
        /* E0 = (ya + yc), E1 = (xa + xc) */
        E = __ADD2(A, C);
        /* F0 = (ya - yc), F1 = (xa - xc) */
        F = __SUB2(A, C);
        t0 = (int16_t) H[0];
        t1 = (int16_t) H[1];
        A = __SRA2(E, ((v2s){ 1, 1 }));
        B = __SRA2(G, ((v2s){ 1, 1 }));
        /* C0 = (xb - xd), C1 = (yd - yb) */
        C = __PACK2(-t1, t0);
        /* D0 = (xd - xb), D1 = (yb - yd) */
        D = __PACK2(t1, -t0);
        /* E0 = (ya+yc) - (yb+yd), E1 = (xa+xc) - (xb+xd) */
        E = __SUB2(E, G);
        /* G1 = (ya-yc) + (xb-xd), G0 = (xa-xc) - (yb-yd) */
        G = __ADD2(F, C);
        /* H1 = (ya-yc) - (xb-xd), H0 = (xa-xc) + (yb-yd) */
        H = __ADD2(F, D);
        /* xc' = (xa-xb+xc-xd)* co2 + (ya-yb+yc-yd)* (si2) */
        /* yc' = (ya-yb+yc-yd)* co2 - (xa-xb+xc-xd)* (si2) */
        t0 = (int16_t)(__DOTP2(CoSi2, E) >> 16U);
        t1 = (int16_t)(__DOTP2(C2, E) >> 16U);
        /* xb' = (xa+yb-xc-yd)* co1 + (ya-xb-yc+xd)* (si1) */
        /* yb' = (ya-xb-yc+xd)* co1 - (xa+yb-xc-yd)* (si1) */
        t2 = (int16_t)(__DOTP2(CoSi1, H) >> 16U);
        t3 = (int16_t)(__DOTP2(C1, H) >> 16U);
        /* xd' = (xa-yb-xc+yd)* Co3 + (ya+xb-yc-xd)* (si3) */
        /* yd' = (ya+xb-yc-xd)* Co3 - (xa-yb-xc+yd)* (si3) */
        t4 = (int16_t)(__DOTP2(CoSi3, G) >> 16U);
        t5 = (int16_t)(__DOTP2(C3, G) >> 16U);
        /* ya' = ya + yb + yc + yd */
        /* xa' = xa + xb + xc + xd */
        A = __ADD2(A, B);
        E = __PACK2(t0, t1);
        F = __PACK2(t2, t3);
        G = __PACK2(t4, t5);
        *((v2s *)&pSrc16[i0 * 2U]) = A;
        *((v2s *)&pSrc16[i1 * 2U]) = E;
        *((v2s *)&pSrc16[i2 * 2U]) = F;
        *((v2s *)&pSrc16[i3 * 2U]) = G;
        /*  Twiddle coefficients index modifier */
        ic = ic + twidCoefModifier;

        queue_domain_destroy(get_alloc_tile(0), data0_queue);
        queue_domain_destroy(get_alloc_tile(1), data1_queue);
        queue_domain_destroy(get_alloc_tile(2), data2_queue);
        queue_domain_destroy(get_alloc_tile(3), data3_queue);
        //queue_destroy(data0_queue);
        //queue_destroy(data1_queue);
        //queue_destroy(data2_queue);
        //queue_destroy(data3_queue);
    }

    /* MIDDLE STAGE */
    /*  Twiddle coefficients index modifier */
    twidCoefModifier <<= 2U;
    /*  Calculation of Middle stage */
    for (k = fftLen / 4U; k > 4U; k >>= 2U) {
        /*  Initializations for the middle stage */
        n1 = n2;
        n2 >>= 2U;
        ic = 0U;
        for (j = 0U; j <= (n2 - 1U); j++) {
            /*  index calculation for the coefficients */
            CoSi1 = *(v2s *)&pCoef16[ic * 2U];
            CoSi2 = *(v2s *)&pCoef16[2U * (ic * 2U)];
            CoSi3 = *(v2s *)&pCoef16[3U * (ic * 2U)];
            t0 = (int16_t) CoSi1[0];
            t1 = (int16_t) CoSi1[1];
            t2 = (int16_t) CoSi2[0];
            t2 = (int16_t) CoSi2[0];
            t3 = (int16_t) CoSi2[1];
            t4 = (int16_t) CoSi3[0];
            t5 = (int16_t) CoSi3[1];
            C1 = __PACK2(-t1, t0);
            C2 = __PACK2(-t3, t2);
            C3 = __PACK2(-t5, t4);
            /*  Twiddle coefficients index modifier */
            ic = ic + twidCoefModifier;
            /*  Butterfly implementation */
            for (i0 = j; i0 < fftLen; i0 += n1) {
              radix4_butterfly_middle( pSrc16, i0, n2,
                                        CoSi1, CoSi2, CoSi3,
                                        C1, C2, C3);
            }
        }
        /*  Twiddle coefficients index modifier */
        twidCoefModifier <<= 2U;
    }
    /* END OF MIDDLE STAGE PROCESSING */

    /* data is in 10.6(q6) format for the 1024 point */
    /* data is in 8.8(q8) format for the 256 point */
    /* data is in 6.10(q10) format for the 64 point */
    /* data is in 4.12(q12) format for the 16 point */
    /*  Initializations for the last stage */
    n1 = n2;
    n2 >>= 2U;
    /* START OF LAST STAGE PROCESSING */
    /*  Butterfly implementation */
    for (i0 = 0U; i0 <= (fftLen - n1); i0 += n1) {
      radix4_butterfly_last(pSrc16, i0, n2);
    }
    /* END OF LAST STAGE PROCESSING */
    /* output is in 11.5(q5) format for the 1024 point */
    /* output is in 9.7(q7) format for the 256 point   */
    /* output is in 7.9(q9) format for the 64 point  */
    /* output is in 5.11(q11) format for the 16 point  */
}
static void mempool_radix4_butterfly_q16p_xpulpimg( int16_t *pSrc16,
                                    uint32_t fftLen,
                                    const int16_t *pCoef16,
                                    uint32_t twidCoefModifier,
                                    uint32_t nPE) {

    v2s CoSi1, CoSi2, CoSi3;
    v2s C1, C2, C3;
    int16_t t0, t1, t2, t3, t4, t5;
    uint32_t n1, n2, ic, i0, j, k;
    uint32_t absolute_core_id = mempool_get_core_id();
    uint32_t core_id = absolute_core_id%nPE;
    uint32_t step, steps;

    /* Total process is divided into three stages */
    /* process first stage, middle stages, & last stage */

    /* Initializations for the first stage */
    n1 = fftLen;
    /* n2 = fftLen/4 */
    n2 = n1 >> 2U;
    step = (n2 + nPE - 1) / nPE;

    /* Input is in 1.15(q15) format */
    /* START OF FIRST STAGE PROCESS */
    for (i0 = core_id * step; i0 < MIN(core_id * step + step, n2); i0++) {

        /*  Twiddle coefficients index modifier */
        ic = i0 * twidCoefModifier;
        /* co1 & si1 are read from Coefficient pointer */
        CoSi1 = *(v2s *)&pCoef16[ic * 2U];
        /* co2 & si2 are read from Coefficient pointer */
        CoSi2 = *(v2s *)&pCoef16[2U * ic * 2U];
        /* co3 & si3 are read from Coefficient pointer */
        CoSi3 = *(v2s *)&pCoef16[3U * (ic * 2U)];
        t0 = (int16_t) CoSi1[0];
        t1 = (int16_t) CoSi1[1];
        t2 = (int16_t) CoSi2[0];
        t3 = (int16_t) CoSi2[1];
        t4 = (int16_t) CoSi3[0];
        t5 = (int16_t) CoSi3[1];
        C1 = __PACK2(-t1, t0);
        C2 = __PACK2(-t3, t2);
        C3 = __PACK2(-t5, t4);
        radix4_butterfly_first( pSrc16, i0, n2,
                                CoSi1, CoSi2, CoSi3,
                                C1, C2, C3);
    }
    mempool_log_barrier(2, absolute_core_id);
    /* data is in 4.11(q11) format */
    /* END OF FIRST STAGE PROCESS */

    /* START OF MIDDLE STAGE PROCESS */
    /*  Twiddle coefficients index modifier */
    twidCoefModifier <<= 2U;
    /*  Calculation of Middle stage */
    for (k = fftLen / 4U; k > 4U; k >>= 2U) {

        uint32_t offset, butt_id;
        n1 = n2;
        n2 >>= 2U;
        step = (n2 + nPE - 1) / nPE;
        butt_id = core_id % n2;
        offset = (core_id / n2) * n1;
        for(j = butt_id * step; j < MIN(butt_id * step + step, n2); j++) {

            /*  Twiddle coefficients index modifier */
            ic = twidCoefModifier * j;
            CoSi1 = *(v2s *)&pCoef16[ic * 2U];
            CoSi2 = *(v2s *)&pCoef16[2U * (ic * 2U)];
            CoSi3 = *(v2s *)&pCoef16[3U * (ic * 2U)];
            t0 = (int16_t) CoSi1[0];
            t1 = (int16_t) CoSi1[1];
            t2 = (int16_t) CoSi2[0];
            t2 = (int16_t) CoSi2[0];
            t3 = (int16_t) CoSi2[1];
            t4 = (int16_t) CoSi3[0];
            t5 = (int16_t) CoSi3[1];
            C1 = __PACK2(-t1, t0);
            C2 = __PACK2(-t3, t2);
            C3 = __PACK2(-t5, t4);

            /*  Butterfly implementation */
            for (i0 = offset + j; i0 < fftLen; i0 += ((nPE + n2 - 1) / n2) * n1) {
              radix4_butterfly_middle(  pSrc16, i0, n2,
                                        CoSi1, CoSi2, CoSi3,
                                        C1, C2, C3);
          }
      }
      /*  Twiddle coefficients index modifier */
      twidCoefModifier <<= 2U;
      mempool_log_barrier(2, absolute_core_id);
    }
    /* END OF MIDDLE STAGE PROCESSING */

    /* data is in 10.6(q6) format for the 1024 point */
    /* data is in 8.8(q8) format for the 256 point */
    /* data is in 6.10(q10) format for the 64 point */
    /* data is in 4.12(q12) format for the 16 point */
    /*  Initializations for the last stage */
    n1 = n2;
    n2 >>= 2U;
    /* START OF LAST STAGE PROCESSING */
    /* start of last stage process */
    steps = fftLen / n1;
    step = (steps + nPE - 1)/nPE;
    /*  Butterfly implementation */
    for (i0 = core_id * step * n1; i0 < MIN((core_id * step + step) * n1, fftLen); i0 += n1) {
      radix4_butterfly_last(pSrc16, i0, n2);
    }
    mempool_log_barrier(2, absolute_core_id);
    /* END OF LAST STAGE PROCESSING */
    /* output is in 11.5(q5) format for the 1024 point */
    /* output is in 9.7(q7) format for the 256 point   */
    /* output is in 7.9(q9) format for the 64 point  */
    /* output is in 5.11(q11) format for the 16 point  */
}
static inline void radix4_butterfly_first( int16_t* pSrc16,
                                            uint32_t i0,
                                            uint32_t n2,
                                            v2s CoSi1,
                                            v2s CoSi2,
                                            v2s CoSi3,
                                            v2s C1,
                                            v2s C2,
                                            v2s C3) {
    v2s A, B, C, D, E, F, G, H;
    int16_t t0, t1, t2, t3, t4, t5;
    uint32_t i1, i2, i3;
    /* index calculation for the input as, */
    /* pSrc16[i0 + 0], pSrc16[i0 + fftLen/4], pSrc16[i0 + fftLen/2], pSrc16[i0 + 3fftLen/4] */
    i1 = i0 + n2;
    i2 = i1 + n2;
    i3 = i2 + n2;
    /* Read yb (real), xb(imag) input */
    B = __SRA2(*(v2s *)&pSrc16[i1 * 2U], ((v2s){ 2, 2 }));
    /* Read yd (real), xd(imag) input */
    D = __SRA2(*(v2s *)&pSrc16[i3 * 2U], ((v2s){ 2, 2 }));
    /* Read ya (real), xa (imag) input */
    A = __SRA2(*(v2s *)&pSrc16[i0 * 2U], ((v2s){ 2, 2 }));
    /* Read yc (real), xc(imag) input */
    C = __SRA2(*(v2s *)&pSrc16[i2 * 2U], ((v2s){ 2, 2 }));
    /* G0 = (yb + yd), G1 = (xb + xd) */
    G = __ADD2(B, D);
    /* H0 = (yb - yd), H1 = (xb - xd) */
    H = __SUB2(B, D);
    /* E0 = (ya + yc), E1 = (xa + xc) */
    E = __ADD2(A, C);
    /* F0 = (ya - yc), F1 = (xa - xc) */
    F = __SUB2(A, C);
    t0 = (int16_t) H[0];
    t1 = (int16_t) H[1];
    A = __SRA2(E, ((v2s){ 1, 1 }));
    B = __SRA2(G, ((v2s){ 1, 1 }));
    /* C0 = (xb - xd), C1 = (yd - yb) */
    C = __PACK2(-t1, t0);
    /* D0 = (xd - xb), D1 = (yb - yd) */
    D = __PACK2(t1, -t0);
    /* E0 = (ya+yc) - (yb+yd), E1 = (xa+xc) - (xb+xd) */
    E = __SUB2(E, G);
    /* G1 = (ya-yc) + (xb-xd), G0 = (xa-xc) - (yb-yd) */
    G = __ADD2(F, C);
    /* H1 = (ya-yc) - (xb-xd), H0 = (xa-xc) + (yb-yd) */
    H = __ADD2(F, D);
    /* xc' = (xa-xb+xc-xd)* co2 + (ya-yb+yc-yd)* (si2) */
    /* yc' = (ya-yb+yc-yd)* co2 - (xa-xb+xc-xd)* (si2) */
    t0 = (int16_t)(__DOTP2(CoSi2, E) >> 16U);
    t1 = (int16_t)(__DOTP2(C2, E) >> 16U);
    /* xb' = (xa+yb-xc-yd)* co1 + (ya-xb-yc+xd)* (si1) */
    /* yb' = (ya-xb-yc+xd)* co1 - (xa+yb-xc-yd)* (si1) */
    t2 = (int16_t)(__DOTP2(CoSi1, H) >> 16U);
    t3 = (int16_t)(__DOTP2(C1, H) >> 16U);
    /* xd' = (xa-yb-xc+yd)* Co3 + (ya+xb-yc-xd)* (si3) */
    /* yd' = (ya+xb-yc-xd)* Co3 - (xa-yb-xc+yd)* (si3) */
    t4 = (int16_t)(__DOTP2(CoSi3, G) >> 16U);
    t5 = (int16_t)(__DOTP2(C3, G) >> 16U);
    /* ya' = ya + yb + yc + yd */
    /* xa' = xa + xb + xc + xd */
    A = __ADD2(A, B);
    E = __PACK2(t0, t1);
    F = __PACK2(t2, t3);
    G = __PACK2(t4, t5);
    *((v2s *)&pSrc16[i0 * 2U]) = A;
    *((v2s *)&pSrc16[i1 * 2U]) = E;
    *((v2s *)&pSrc16[i2 * 2U]) = F;
    *((v2s *)&pSrc16[i3 * 2U]) = G;
}
static inline void radix4_butterfly_middle(   int16_t* pSrc16,
                                              uint32_t i0,
                                              uint32_t n2,
                                              v2s CoSi1,
                                              v2s CoSi2,
                                              v2s CoSi3,
                                              v2s C1,
                                              v2s C2,
                                              v2s C3) {
    v2s A, B, C, D, E, F, G, H;
    int16_t t0, t1, t2, t3, t4, t5;
    uint32_t i1, i2, i3;

    /*  index calculation for the input as, */
    /*  pSrc16[i0 + 0], pSrc16[i0 + fftLen/4], pSrc16[i0 + fftLen/2], pSrc16[i0 +
     * 3fftLen/4] */
    i1 = i0 + n2;
    i2 = i1 + n2;
    i3 = i2 + n2;
    /* Read yb (real), xb(imag) input */
    B = *(v2s *)&pSrc16[i1 * 2U];
    /* Read yd (real), xd(imag) input */
    D = *(v2s *)&pSrc16[i3 * 2U];
    /* Read ya (real), xa(imag) input */
    A = *(v2s *)&pSrc16[i0 * 2U];
    /* Read yc (real), xc(imag) input */
    C = *(v2s *)&pSrc16[i2 * 2U];
    /* G0 = (yb + yd), G1 = (xb + xd) */
    G = __ADD2(B, D);
    /* H0 = (yb - yd), H1 = (xb - xd) */
    H = __SUB2(B, D);
    /* E0 = (ya + yc), E1 = (xa + xc) */
    E = __ADD2(A, C);
    /* F0 = (ya - yc), F1 =(xa - xc) */
    F = __SUB2(A, C);
    G = __SRA2(G, ((v2s){ 1, 1 }));
    H = __SRA2(H, ((v2s){ 1, 1 }));
    E = __SRA2(E, ((v2s){ 1, 1 }));
    F = __SRA2(F, ((v2s){ 1, 1 }));
    t0 = (int16_t) H[0];
    t1 = (int16_t) H[1];
    /* C0 = (ya+yc) - (yb+yd), C1 = (xa+xc) - (xb+xd) */
    C = __SUB2(E, G);
    /* D0 = (ya+yc) + (yb+yd), D1 = (xa+xc) + (xb+xd) */
    D = __ADD2(E, G);
    /* A0 = (xb-xd), A1 = (yd-yb) */
    A = __PACK2(t1, -t0);
    /* B0 = (xd-xb), B1 = (yb-yd) */
    B = __PACK2(-t1, t0);
    /* xa' = xa + xb + xc + xd */
    /* ya' = ya + yb + yc + yd */
    *((v2s *)&pSrc16[i0 * 2U]) = __SRA2(D, ((v2s){ 1, 1 }));
    /* E1 = (ya-yc) + (xb-xd),  E0 = (xa-xc) - (yb-yd)) */
    E = __ADD2(F, A);
    /* F1 = (ya-yc) - (xb-xd), F0 = (xa-xc) + (yb-yd)) */
    F = __ADD2(F, B);
    /* xc' = (xa-xb+xc-xd)* co2 + (ya-yb+yc-yd)* (si2) */
    /* yc' = (ya-yb+yc-yd)* co2 - (xa-xb+xc-xd)* (si2) */
    t0 = (int16_t)(__DOTP2(CoSi2, C) >> 16U);
    t1 = (int16_t)(__DOTP2(C2, C) >> 16U);
    /* xb' = (xa+yb-xc-yd)* co1 + (ya-xb-yc+xd)* (si1) */
    /* yb' = (ya-xb-yc+xd)* co1 - (xa+yb-xc-yd)* (si1) */
    t2 = (int16_t)(__DOTP2(CoSi1, F) >> 16U);
    t3 = (int16_t)(__DOTP2(C1, F) >> 16U);
    /* xd' = (xa-yb-xc+yd)* Co3 + (ya+xb-yc-xd)* (si3) */
    /* yd' = (ya+xb-yc-xd)* Co3 - (xa-yb-xc+yd)* (si3) */
    t4 = (int16_t)(__DOTP2(CoSi3, E) >> 16U);
    t5 = (int16_t)(__DOTP2(C3, E) >> 16U);
    A = __PACK2(t0, t1);
    B = __PACK2(t2, t3);
    C = __PACK2(t4, t5);
    *((v2s *)&pSrc16[i1 * 2U]) = A;
    *((v2s *)&pSrc16[i2 * 2U]) = B;
    *((v2s *)&pSrc16[i3 * 2U]) = C;
}
static inline void radix4_butterfly_last(  int16_t* pSrc16,
                                            uint32_t i0,
                                            uint32_t n2) {
    v2s A, B, C, D, E, F, G, H;
    int16_t t0, t1;
    uint32_t i1, i2, i3;

    /*  index calculation for the input as, */
    /*  pSrc16[i0 + 0], pSrc16[i0 + fftLen/4],
        pSrc16[i0 + fftLen/2], pSrc16[i0 + 3fftLen/4] */
    i1 = i0 + n2;
    i2 = i1 + n2;
    i3 = i2 + n2;
    /* Read yb (real), xb(imag) input */
    B = *(v2s *)&pSrc16[i1 * 2U];
    /* Read yd (real), xd(imag) input */
    D = *(v2s *)&pSrc16[i3 * 2U];
    /* Read ya (real), xa(imag) input */
    A = *(v2s *)&pSrc16[i0 * 2U];
    /* Read yc (real), xc(imag) input */
    C = *(v2s *)&pSrc16[i2 * 2U];
    /* H0 = (yb-yd), H1 = (xb-xd) */
    H = __SUB2(B, D);
    /* G0 = (yb+yd), G1 = (xb+xd) */
    G = __ADD2(B, D);
    /* E0 = (ya+yc), E1 = (xa+xc) */
    E = __ADD2(A, C);
    /* F0 = (ya-yc), F1 = (xa-xc) */
    F = __SUB2(A, C);
    H = __SRA2(H, ((v2s){ 1, 1 }));
    G = __SRA2(G, ((v2s){ 1, 1 }));
    E = __SRA2(E, ((v2s){ 1, 1 }));
    t0 = (int16_t) H[0];
    t1 = (int16_t) H[1];
    F = __SRA2(F, ((v2s){ 1, 1 }));
    /* xa' = (xa+xb+xc+xd) */
    /* ya' = (ya+yb+yc+yd) */
    *((v2s *)&pSrc16[i0 * 2U]) = __ADD2(E, G);
    /* A0 = (xb-xd), A1 = (yd-yb) */
    A = __PACK2(t1, -t0);
    /* B0 = (xd-xb), B1 = (yb-yd) */
    B = __PACK2(-t1, t0);
    /* xc' = (xa-xb+xc-xd) */
    /* yc' = (ya-yb+yc-yd) */
    E = __SUB2(E, G);
    /* xb' = (xa+yb-xc-yd) */
    /* yb' = (ya-xb-yc+xd) */
    A = __ADD2(F, A);
    /* xd' = (xa-yb-xc+yd) */
    /* yd' = (ya+xb-yc-xd) */
    B = __ADD2(F, B);
    *((v2s *)&pSrc16[i1 * 2U]) = E;
    *((v2s *)&pSrc16[i2 * 2U]) = A;
    *((v2s *)&pSrc16[i3 * 2U]) = B;
}
