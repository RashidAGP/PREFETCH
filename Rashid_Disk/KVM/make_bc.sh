#sudo sh -c 'echo 1 >/proc/sys/kernel/perf_event_paranoid'
../../build/X86/gem5.fast --outdir=/home/rashid/new_benchmarks/bc --redirect-stdout --redirect-stderr ../../configs/deprecated/example/fs.py \
    --take-checkpoint=34794000000000,2000000000000 \
    --script=/home/rashid/all_scripts/gapbs/bc.rcS \
    --kernel=/home/rashid/vmlinux/vmlinux-5.4.49 \
    --disk-image=/home/rashid/fs_img/gapbs_bc.img \
    --cpu-type=X86KvmCPU \
    --mem-size=16GB \
    --ruby \
    --network=garnet \
    -n 1 \
    --caches \
    --l1d_size=64kB \
    --l1i_size=64kB \
    --l1d_assoc=8 \
    --num-l2caches=1 \
    --l2_size=2MB \
    --l2_assoc=16 \
    --cpu-clock=2GHz \
    --cacheline_size=64
