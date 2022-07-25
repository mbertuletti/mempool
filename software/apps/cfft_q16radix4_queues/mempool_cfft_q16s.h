// Copyright 2022 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Author: Marco Bertuletti, ETH Zurich

static void mempool_cfft_q16s(  uint16_t fftLen,
                                int16_t *pTwiddle,
                                uint16_t *pBitRevTable,
                                int16_t *pSrc,
                                uint16_t bitReverseLen,
                                uint8_t ifftFlag,
                                uint8_t bitReverseFlag);

static void mempool_cfft_radix4by2_q16s(  int16_t *pSrc,
                                          uint32_t fftLen,
                                          const int16_t *pCoef);

void mempool_cfft_q16s( uint16_t fftLen,
                        int16_t *pTwiddle,
                        uint16_t *pBitRevTable,
                        int16_t *pSrc,
                        uint16_t bitReverseLen,
                        uint8_t ifftFlag,
                        uint8_t bitReverseFlag) {

    if (ifftFlag == 0) {
        switch (fftLen) {
        case 16:
        case 64:
        case 256:
        case 1024:
        case 4096:
            mempool_radix4_butterfly_q16s_xpulpimg(pSrc, fftLen, pTwiddle, 1);
            break;
        case 32:
        case 128:
        case 512:
        case 2048:
            mempool_cfft_radix4by2_q16s(pSrc, fftLen, pTwiddle);
            break;
        }
    }

    if (bitReverseFlag)
        mempool_bitrev_q16s_riscv32((uint16_t *)pSrc, bitReverseLen, pBitRevTable);
}

void mempool_cfft_radix4by2_q16s( int16_t *pSrc,
                                  uint32_t fftLen,
                                  const int16_t *pCoef) {

    uint32_t i;
    uint32_t n2;
    v2s pa, pb;

    uint32_t l;
    v2s CoSi;
    v2s a, b, t;
    int16_t testa, testb;

    n2 = fftLen >> 1;

    for (i = 0; i < n2; i++) {
        CoSi = *(v2s *)&pCoef[i * 2];
        l = i + n2;
        a = __SRA2(*(v2s *)&pSrc[2 * i], ((v2s){ 1, 1 }));
        b = __SRA2(*(v2s *)&pSrc[2 * l], ((v2s){ 1, 1 }));
        t = __SUB2(a, b);
        *((v2s *)&pSrc[i * 2]) = __SRA2(__ADD2(a, b), ((v2s){ 1, 1 }));

        testa = (int16_t)(__DOTP2(t, CoSi) >> 16);
        testb = (int16_t)(__DOTP2(t, __PACK2(-CoSi[1], CoSi[0])) >> 16);
        *((v2s *)&pSrc[l * 2]) = __PACK2(testa, testb);
    }

    // first col
    mempool_radix4_butterfly_q16s_xpulpimg(pSrc, n2, (int16_t *)pCoef, 2U);
    // second col
    mempool_radix4_butterfly_q16s_xpulpimg(pSrc + fftLen, n2, (int16_t *)pCoef, 2U);

    for (i = 0; i < (fftLen >> 1); i++) {
        pa = *(v2s *)&pSrc[4 * i];
        pb = *(v2s *)&pSrc[4 * i + 2];

        pa = __SLL2(pa, ((v2s){ 1, 1 }));
        pb = __SLL2(pb, ((v2s){ 1, 1 }));

        *((v2s *)&pSrc[4 * i]) = pa;
        *((v2s *)&pSrc[4 * i + 2]) = pb;
    }
}
