#!/bin/bash
# Final race trajectory optimizer

echo "=== Race Trajectory Optimizer ==="
echo "Using TUMFTM functions with cvxopt backend"
echo

# Default to Berlin track
TRACK=${1:-berlin_2018}

echo "Optimizing track: $TRACK"
echo

# Run optimizer
docker run -it --rm \
    -v $(pwd)/src:/app/src \
    -v $(pwd)/config:/app/config \
    -v $(pwd)/inputs:/app/inputs \
    -v $(pwd)/outputs:/app/outputs \
    -v $(pwd)/global_racetrajectory_optimization:/app/global_racetrajectory_optimization \
    race-trajectory-optimizer-tumftm \
    python /app/src/working_optimizer_final.py

echo
echo "Results saved to outputs/"
ls -la outputs/*.png outputs/*.csv 2>/dev/null

# Display image if possible
if command -v xdg-open &> /dev/null; then
    xdg-open outputs/racing_line_analysis.png 2>/dev/null || true
fi
