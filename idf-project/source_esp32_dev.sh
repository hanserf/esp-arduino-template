#!/bin/bash
source ${MY_IDF_PATH}/export.sh
export PYTHONPATH="$MY_IDF_PATH/tools:$MY_IDF_PATH/tools/ci/python_packages:$PYTHONPATH"
