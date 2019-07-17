#!/bin/bash

function do_local_trick_cp(){ 
    local_dir=$1
    cd $local_dir
    echo "Compiling source codes in: $local_dir"
    make clean
    trick-CP
    make_err=$?
    return $make_err 
}

function do_local_trick_cp_clean(){ 
    local_dir=$1
    top_dir=$2
    cd $local_dir
    echo "Clean source codes in: $local_dir"
    make clean
    cd $top_dir
    $top_dir/trick_deep_clean.sh $local_dir
    make_err=$?
    return $make_err 
} 

function do_local_make(){ 
    local_dir=$1
    cd $local_dir
    echo "Compiling source codes in: $local_dir"
    make
    make_err=$?
    return $make_err 
}

function do_local_clean(){ 
    local_dir=$1
    cd $local_dir
    echo "Clean source codes in: $local_dir"
    make clean
    make_err=$?
    return $make_err 
} 

top_dir=$1

build_system_mode=$2
echo "INFO: $top_dir"
echo "INFO: $build_system_mode"
shift 2
# Loop for all MODULES 

file_list=( "$@" )
for var in "${file_list[@]}"
do
    echo "Compiling $var"
    target_dir=$var
    case $build_system_mode in
        trick-build)
            do_local_trick_cp $var
            ;;
        trick-clean)
            do_local_trick_cp_clean $var $top_dir
            ;;
        module-build)
            do_local_make $var
            ;;
        module-clean)
            do_local_clean $var
            ;;

        *) echo "$build_system : unknown."
    esac
    error_code=$? 
    if [ $error_code -ne 0 ]; then exit $error_code; fi 
done