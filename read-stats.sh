#!/bin/bash

filename=$1
echo "stats name $filename"

# for icache
echo "icache" > temp.txt
echo "===" >> temp.txt
awk '/icache.tags.same_bits_hit_distr::[0-9].*/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt
awk '/icache.tags.same_bits_mis_distr::[0-9].*/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt
awk '/icache.tags.diff_bit_freq::[0-9].*/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt
awk '/icache.tags.off_ways::[0-9].*/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt
awk '/icache.tags.fake_misses::[0-9].*/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt
awk '/icache.tags.fake_hits/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt
awk '/icache.tags.set_reads/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt

# for dcache
echo "dcache" >> temp.txt
echo "===" >> temp.txt
awk '/dcache.tags.same_bits_hit_distr::[0-9].*/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt
awk '/dcache.tags.same_bits_mis_distr::[0-9].*/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt
awk '/dcache.tags.diff_bit_freq::[0-9].*/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt
awk '/dcache.tags.off_ways::[0-9].*/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt
awk '/dcache.tags.fake_misses::[0-9].*/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt
awk '/dcache.tags.fake_hits/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt
awk '/dcache.tags.set_reads/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt

# for l2cache
echo "l2cache" >> temp.txt
echo "===" >> temp.txt
awk '/l2.tags.same_bits_hit_distr::[0-9].*/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt
awk '/l2.tags.same_bits_mis_distr::[0-9].*/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt
awk '/l2.tags.diff_bit_freq::[0-9].*/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt
awk '/l2.tags.off_ways::[0-9].*/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt
awk '/l2.tags.fake_misses::[0-9].*/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt
awk '/l2.tags.fake_hits/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt
awk '/l2.tags.set_reads/{print $2}' ${filename} >> temp.txt
echo "===" >> temp.txt

tmpstr=''

rm distr.txt

while read line
do
	if [[ "$line" == "cache" ]];then
		echo "${line}" >> distr.txt
	elif [[ "$line" == "===" ]];then
		echo "${tmpstr}" >> distr.txt
		tmpstr=''
	else
		tmpstr="${tmpstr} ${line}"
	fi
done < temp.txt

