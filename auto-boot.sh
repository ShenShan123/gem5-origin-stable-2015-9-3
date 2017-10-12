#!/bin/bash

../build/x86/gem5.fast ../configs/example/se.py \
--mem-size=1024MB \
--caches --l1i_size=32kB --l1d_size=32kB --l1d_assoc=8 --l1i_assoc=8 --l2_assoc=16 --l2cache --l2_size=128kB --num-l2caches=1 \
--cpu-type=arm_detailed \
-c ./bzip2_base.gcc41-amd64bit -o "chicken.jpg 30"
