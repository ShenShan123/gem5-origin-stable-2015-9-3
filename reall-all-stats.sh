#!/bin/bash


for dir in ./*
do
	if [[ -d ${dir} ]]
	then
		echo "$dir"
		cd ${dir}
		cp ../read-stats.sh ./
		./read-stats.sh stats.txt
		mv distr.txt ${dir}-distr.txt
		cd ../
	fi
done 