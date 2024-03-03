gdb --args ./build/X86/gem5.debug \
    --debug-flags=DRAMsim3,DRAMPower \
    --outdir=/home/rashid/UAC/4cores/st_raw_timing \
    ./configs/deprecated/example/fs.py \
    --checkpoint-dir=/home/rashid/checkpoints/tc_raw -r 1\
    -r 1\
    --kernel=/home/rashid/vmlinux/vmlinux-5.4.49 \
    --disk-image=/home/rashid/fs_img/linux-x86.img \
    --cpu-type=DerivO3CPU \
    --restore-with-cpu=TimingSimpleCPU \
    --standard-switch 1 \
    --mem-size=16GB \
    --network=garnet\
    --ruby \
    --caches \
    --l1d_size=64kB \
    --l1i_size=64kB \
    --l1d_assoc=8 \
    --l1i_assoc=8 \
    -n 4 \
    --num-l2caches=4 \
    --num-dirs=4 \
    --topology=Mesh_XY \
    --mesh-rows=2 \
    --l2_size=2MB \
    --l2_assoc=16 \
    --cpu-clock=2GHz \
    --cacheline_size=64 \
    -I 10000000
# -I 500000000
