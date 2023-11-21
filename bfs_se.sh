gdb --args ./build/X86/gem5.debug --outdir=/home/rashid/pagerank_result ./configs/deprecated/example/se.py \
    --cmd=/home/rashid/benchmark/CRONO/apps/bfs/bfs \
    --cpu-type=TimingSimpleCPU \
    --mem-size=8GB \
    --caches \
    --l1d_size=64kB \
    --l1i_size=32kB \
    --l1d_assoc=8 \
    --l1i_assoc=8 \
    --num-l2caches=1 \
    --l2_size=2MB \
    --l2_assoc=16 \
    --cpu-clock=2GHz \
    --cacheline_size=64 \
    -I 500000000
