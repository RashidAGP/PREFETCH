#!/bin/sh
#SBATCH -A naiss2023-5-106
#SBATCH -p core
#SBATCH -n 4
#SBATCH -t 02:30:00
#SBATCH -J compile_raw_gem5


module purge


# Add the path to Python 3.7.4 before the system's default Python in PATH

# load module for scons
echo "load PROJ/8.1.0"
module load PROJ/8.1.0

# choose one of these two versions as they use GCCcore-8.3.0
# need this so that pkg-config can detect hdf5 and add linker flags
echo "load HDF5"
module load HDF5/1.10.5-gompi-2019b
module load HDF5/1.10.5-iimpi-2019b

#echo "load pkg-config"
#module load pkg-config/0.29.2-GCCcore-8.3.0

echo "load pkg-config"
module load pkg-config/0.29.2-GCCcore-8.3.0


echo "load Scons/3.1.1"
#module load SCons/3.1.1-GCCcore-8.3.0
module load SCons/3.1.1-GCCcore-8.3.0
# if GCCCore/8.3.0 does not work (saying undefined instructions / Can't find working python version) load 10.3.0
#echo "load gcc"
#module load gcc/10.3.0


which python3-config
PYTHON3_CONFIG=$(which python3-config)

echo "check hdf5 with pkg-config"
pkg-config --cflags-only-I --libs-only-L hdf5
pkg-config --cflags-only-I --libs-only-L hdf5-serial
# It turns out setting that env var only works for the commands in this script but not for SCons

module load GCC/10.3.0
echo "gcc version"
gcc --version



echo "python version"
module load python3/3.6.8
python3 --version



scons PYTHON_CONFIG=$PYTHON3_CONFIG PROTOCOL=MESI_Two_Level build/X86/gem5.fast -j 40

echo "FINISHED ! "
