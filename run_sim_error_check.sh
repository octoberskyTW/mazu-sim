#!/bin/bash
TOP_DIR=$1
PROJECT=$2
S_DEFINE_PATH=$TOP_DIR/sim_exe/egse_dm

case $PROJECT in
    "egse_dm")
        cd $S_DEFINE_PATH
        python3 $TOP_DIR/scripts/generate_error.py $TOP_DIR/tables/golden_answer/golden.csv $S_DEFINE_PATH/RUN_golden/log_rocket_csv.csv -l
        python3 $TOP_DIR/scripts/ci_test.py $S_DEFINE_PATH/result.csv 5e-5 | tee test_result
        test ${PIPESTATUS[0]} -eq 0
        ;;
    *)
        echo "There is no need to check"
        exit 0
        ;;
esac