#!/bin/sh
# Builds the three host simulation binaries (see README.md):
#   sim      - operational firmware (src/main.cpp as-is)
#   simdiag  - diagnostic build (-DDIAG_MODE), scenarios 20-23
#   simct    - operational firmware with CT_READY=1, scenarios 30-31
#   simui    - UI screenshots through a virtual ILI9163C (ui_*.ppm)
#   simlive / simlive_diag - interactive real-time simulator (see live.py)
set -e
cd "$(dirname "$0")"
CXX="${CXX:-c++}"
FLAGS="-std=c++17 -O1 -Wall -Wno-unused-function -I mock"
$CXX $FLAGS            -o sim     sim.cpp
$CXX $FLAGS -DDIAG_MODE -o simdiag sim.cpp
$CXX $FLAGS -DCT_READY=1 -o simct sim_ct.cpp
$CXX $FLAGS            -o simui  sim_ui.cpp
$CXX $FLAGS            -o simlive      sim_live.cpp
$CXX $FLAGS -DDIAG_MODE -o simlive_diag sim_live.cpp
echo "built: sim simdiag simct simui simlive simlive_diag"
