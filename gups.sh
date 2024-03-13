./build/X86/gem5.fast \
    --outdir=/home/rashid/UAC/test_gups \
    --redirect-stdout \
    --redirect-stderr \
    ./configs/deprecated/example/fs.py \
    --checkpoint-dir=/home/rashid/new_benchmarks/crono_gups_raw -r 2\
    --kernel=/home/rashid/vmlinux/vmlinux-5.4.49 \
    --disk-image=/home/rashid/fs_img/gups_2.img \
    --cpu-type=DerivO3CPU \
    --restore-with-cpu=DerivO3CPU \
    --mem-size=16GB \
    --network=garnet\
    --ruby \
    --caches \
    --l1d_size=64kB \
    --l1i_size=64kB \
    --l1d_assoc=8 \
    --l1i_assoc=8 \
    -n 1 \
    --num-l2caches=1 \
    --num-dirs=1 \
    --l2_size=2MB \
    --l2_assoc=16 \
    --cpu-clock=2GHz \
    --cacheline_size=64 \
    -I 100000000
# -I 500000000
