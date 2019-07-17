#!/bin/bash
TOP_DIR=$1

cd $TOP_DIR/sim_exe/sample_code
./S_main_*.exe RUN_test/input.cpp &

cd $TOP_DIR/sim_exe/sample_client
./S_main_*.exe RUN_test/input.cpp