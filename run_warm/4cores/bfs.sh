../../build/X86/gem5.fast --outdir=/home/rashid/warm_results/4cores/bfs_raw --redirect-stdout --redirect-stderr ../../configs/deprecated/example/fs.py \
    --checkpoint-dir=/home/rashid/checkpoints_rackham/v23/4cores/bfs -r 1\
    --kernel=/home/rashid/vmlinux/vmlinux-5.4.49 \
    --disk-image=/home/rashid/fs_img/x86-graphs.img \
    --cpu-type=DerivO3CPU \
    --restore-with-cpu=DerivO3CPU \
    --mem-size=8GB \
    --ruby \
    --l2cache \
    -n 4 \
    --num-cpus=4 \
    --num-l2cache=4 \
    --topology=Mesh_XY \
    --mesh-rows=2 \
    --num-dirs=4 \
    --l1d_size=64kB \
    --l1i_size=64kB \
    --l1d_assoc=8 \
    --l1i_assoc=8 \
    --l2_size=2MB \
    --l2_assoc=16 \
    --cpu-clock=2GHz \
    --cacheline_size=64 \
    -I 450000000
