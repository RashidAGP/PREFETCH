./build/X86/gem5.opt \
    --debug-flags=RubySequencer,RubySlicc,RubyGenerated,RubyCache \
    --outdir=/home/rashid/warm_result/gapbs/kron/bc_prefetch \
    --redirect-stdout \
    --redirect-stderr \
    ./configs/deprecated/example/fs.py \
    --checkpoint-dir=/home/rashid/new_benchmarks/kron/without_garnet/bc \
    -r 1\
    --kernel=/home/rashid/vmlinux/vmlinux-5.4.49 \
    --disk-image=/home/rashid/fs_img/gapbs_kron.img \
    --cpu-type=TimingSimpleCPU \
    --restore-with-cpu=TimingSimpleCPU \
    --mem-size=16GB \
    --mem-type=DRAMsim3\
    --network=simple\
    --ruby \
    --l1d_size=64kB \
    --l1i_size=64kB \
    --l1d_assoc=8 \
    --l1i_assoc=8 \
    --l2_size=2MB \
    --l2_assoc=16 \
    --cpu-clock=2GHz \
    --cacheline_size=64 \
    -I 500000
# -I 500000000
