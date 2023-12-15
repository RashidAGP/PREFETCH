./build/X86/gem5.opt \
    --debug-flags=RubyTest,RubySlicc,RubyGenerated \
    --outdir=/home/rashid/warm_result/gapbs/kron/bc_prefetch \
    --redirect-stdout \
    --redirect-stderr \
    ./configs/example/ruby_random_test.py \
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
    --cacheline_size=64
# -I 500000000
