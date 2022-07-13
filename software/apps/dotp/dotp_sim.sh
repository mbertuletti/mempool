cd /scratch2/mbertuletti/mempool/hardware

for i in 128 256
do
    DEFINES+=-DLEN=$((1024 * $i)) \
    make dotp -C ../software/apps
    app=dotp make simcvcs
    make trace
done

cd /scratch2/mbertuletti/mempool/software/apps/dotp
