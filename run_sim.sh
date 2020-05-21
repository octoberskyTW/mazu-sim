#!/bin/bash
TOP_DIR=$1
PROJECT=$2

case $PROJECT in
    "sample")
        cd $TOP_DIR/sim_exe/sample_master
        ./S_main_*.exe RUN_test/input.cpp &

        cd $TOP_DIR/sim_exe/sample_slave
        ./S_main_*.exe RUN_test/input.cpp
        ;;
    "skyline")
        cd $TOP_DIR/sim_exe/skyline_dm
        ./S_main_*.exe RUN_golden/input_dm.cpp &

        cd $TOP_DIR/sim_exe/skyline_gnc
        ./S_main_*.exe RUN_golden/input_gnc.cpp
        ;;
    *)
        echo "Pleas check the PROJECT name !!"
        exit 1
        ;;
esac