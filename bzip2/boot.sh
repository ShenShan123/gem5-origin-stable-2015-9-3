#!/bin/bash

export M5_PATH=/mnt/hgfs/ShareShen/x86_arch

./../build/x86/gem5.fast --outdir=./m5out-se-x86 ./../configs/example/se.py \
 -c ./bzip2_base.gcc41-amd64bit \
 -o "./chicken.jpg 30" \
 -n 1 --cpu-type=DerivO3CPU --caches --cacheline_size=64 \
 --l1i_size=32kB --l1d_size=32kB --l1d_assoc=4 --l1i_assoc=4 --l2cache --l2_size=2MB --l2_assoc=8 \
 --abs-max-tick=9422075500
