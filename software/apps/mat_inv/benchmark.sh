# Copyright 2021 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

# Author: Marco Bertuletti, ETH Zurich

for idx in 1024 512 256 128 64
do
    cd /scratch2/mbertuletti/mempool/software/apps
    DEFINES+=-DN_USED_BANKS=$idx \
    make mat_inv
    #cd /scratch2/mbertuletti/mempool/hardware
    #app=mat_inv make simcvcs
    #make trace
    #cd /scratch2/mbertuletti/mempool/software/apps/mat_inv
done
