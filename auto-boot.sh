#!/bin/bash

../build/x86/gem5.fast ../configs/example/se.py \
--fastmem \
--cpu-type=AtomicSimpleCPU \
--simpoint-profile --simpoint-interval=10000000 \
-c ./tonto_base.gcc41-amd64bit
#-c ./perlbench_base.gcc41-amd64bit -o "-I./lib checkspam.pl 2500 5 25 11 150 1 1 1 1"
#-c ./bwaves_base.gcc41-amd64bit
#-c ./bzip2_base.gcc41-amd64bit -o "chicken.jpg 30"
#
# take checkpoints from Simpoint files
../build/x86/gem5.fast ../configs/example/se.py \
--mem-size=1024MB --fastmem \
--cpu-type=AtomicSimpleCPU \
--take-simpoint-checkpoint=../SimPoint.3.2/output/tonto.simpoints,../SimPoint.3.2/output/tonto.weights,1,0 \
-c ./tonto_base.gcc41-amd64bit
# restore from the simpoint checkpoints
../build/x86/gem5.fast ../configs/example/se.py \
--caches --l1i_size=32kB --l1i_assoc=8 --l1d_size=32kB --l1d_assoc=8 \
--l2cache --l2_assoc=8 --l2_size=4MB --num-l2caches=1 \
--mem-size=1024MB \
--cpu-type=DerivO3CPU \
-r 2099998889 --at-instruction \
-c ./tonto_base.gcc41-amd64bit

../build/x86/gem5.fast ../configs/example/se.py \
--caches --l1i_size=32kB --l1i_assoc=4 --l1d_size=32kB --l1d_assoc=4 \
--l2cache --l2_assoc=16 --l2_size=4MB --num-l2caches=1 \
--mem-size=1024MB \
--cpu-type=DerivO3CPU \
-r 2099998889 --at-instruction \
-c ./tonto_base.gcc41-amd64bit