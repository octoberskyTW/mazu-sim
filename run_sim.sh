#!/bin/bash
TOP_DIR=$1
PROJECT=$2

case $PROJECT in
    "sample_code")
        cd $TOP_DIR/sim_exe/sample_code
        ./S_main_*.exe RUN_test/input.cpp &

        cd $TOP_DIR/sim_exe/sample_client
        ./S_main_*.exe RUN_test/input.cpp
        ;;
    "egse_dm")
        cd $TOP_DIR/sim_exe/egse_dm
        ./S_main_*.exe RUN_golden/input_dm.cpp &

        cd $TOP_DIR/sim_exe/fsw_gnc
        ./S_main_*.exe RUN_golden/input_gnc.cpp
        ;;
    *)
        echo "Pleas check the PROJECT name !!"
        exit 1
        ;;
esac