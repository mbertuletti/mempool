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
