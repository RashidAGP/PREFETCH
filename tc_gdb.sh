./build/X86/gem5.debug \
    --outdir=/home/rashid/warm_result/gapbs/kron/tc_prefetch \
    --redirect-stdout \
    --redirect-stderr \
    ./configs/deprecated/example/fs.py \
    --checkpoint-dir=/home/rashid/new_benchmarks/kron/tc \
    -r 1\
    --kernel=/home/rashid/vmlinux/vmlinux-5.4.49 \
    --disk-image=/home/rashid/fs_img/gapbs_kron.img \
    --cpu-type=DerivO3CPU \
    --restore-with-cpu=DerivO3CPU \
    --mem-size=16GB \
    --mem-type=DRAMsim3\
    --ruby \
    --network=garnet \
    --caches \
    --l1d_size=64kB \
    --l1i_size=64kB \
    --l1d_assoc=8 \
    --l1i_assoc=8 \
    --num-l2caches=1 \
    --num-dirs=1 \
    --l2_size=2MB \
    --l2_assoc=16 \
    --cpu-clock=1GHz \
    --cacheline_size=64 \
    -I 10000000
# -I 500000000
