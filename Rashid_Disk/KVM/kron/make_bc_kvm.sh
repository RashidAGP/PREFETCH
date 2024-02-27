#sudo sh -c 'echo 1 >/proc/sys/kernel/perf_event_paranoid'
../../../build/X86/gem5.fast --outdir=/home/rashid/new_benchmarks/kron/bc_kvm --redirect-stdout --redirect-stderr ../../../configs/deprecated/example/fs.py \
    --script=/home/rashid/all_scripts/gapbs/kron/bc.rcS \
    --kernel=/home/rashid/vmlinux/vmlinux-5.4.49 \
    --disk-image=/home/rashid/fs_img/gapbs_kron.img \
    --cpu-type=X86KvmCPU \
    --mem-size=16GB \
    --ruby \
    --network=garnet \
    -n 4 \
    --caches \
    --l1d_size=64kB \
    --l1i_size=64kB \
    --l1d_assoc=8 \
    --num-l2caches=4 \
    --l2_size=2MB \
    --l2_assoc=16 \
    --cpu-clock=2GHz \
    --cacheline_size=64
