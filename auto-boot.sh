#!/bin/bash

../build/x86/gem5.fast ../configs/example/se.py \
--caches --l1i_size=32kB --l1i_assoc=8 --l1d_size=32kB --l1d_assoc=8 \
--l2cache --l2_assoc=8 --l2_size=1MB --num-l2caches=1 \
--mem-size=1024MB \
--cpu-type=arm_detailed \
-c ./tonto_base.gcc41-amd64bit
#-c ./perlbench_base.gcc41-amd64bit -o "-I./lib checkspam.pl 2500 5 25 11 150 1 1 1 1"
#-c ./bwaves_base.gcc41-amd64bit
#-c ./bzip2_base.gcc41-amd64bit -o "chicken.jpg 30"
#