#!/bin/bash

function do_local_trick_cp(){ 
    local_dir=$1
    cd $local_dir
    echo "Compiling source codes in: $local_dir"
    trick-CP
    make_err=$?
    return $make_err 
}

function do_local_trick_cp_clean(){ 
    local_dir=$1
    cd $local_dir
    echo "Clean source codes in: $local_dir"
    make clean
    make_err=$?
    return $make_err 
} 

do_build=$1
shift 1
# Loop for all MODULES 
for var in "$@" 
    do 
    echo "Compiling $var" 
    target_dir=$var
    if [ $do_build -eq 1 ]; then
        do_local_trick_cp $target_dir
    else
        do_local_trick_cp_clean $target_dir
    fi
    error_code=$? 
    if [ $error_code -ne 0 ]; then exit $error_code; fi 
done 