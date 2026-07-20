#!/bin/bash
# Runs evo_ape on a 150s-180s (elapsed) window, correctly converting
# to absolute timestamps first since EuRoC/TUM files use epoch-scale time.

set -e

EVO_APE=~/miniconda3/envs/vigs/bin/evo_ape
GT_FILE=data.tum
EST_FILE=../../sid/gtsam_custom_vio/results/learned_vio_MH01.tum 
ELAPSED_START=0
ELAPSED_END=600

# first timestamp in the ground truth file (T0)
T0=$(head -1 "$GT_FILE" | awk '{print $1}')

# compute absolute bounds using python (handles the scientific notation safely)
T_START=$(python3 -c "print(${T0} + ${ELAPSED_START})")
T_END=$(python3 -c "print(${T0} + ${ELAPSED_END})")

echo "T0 (trajectory start)   = $T0"
echo "t_start (absolute)      = $T_START"
echo "t_end   (absolute)      = $T_END"
echo ""

# "$EVO_APE" tum "$GT_FILE" "$EST_FILE" -a --t_start "$T_START" --t_end "$T_END" --plot
"$EVO_APE" tum "$GT_FILE" "$EST_FILE" -a --t_start "$T_START" --t_end "$T_END" --plot --save_plot trajectory_plot.pdf