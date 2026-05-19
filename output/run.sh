#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"
export LD_LIBRARY_PATH=../lib:${LD_LIBRARY_PATH}

./cw_DealRCF_nebulalink ../config/DealRCF.cfg
