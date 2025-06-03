#!/bin/bash
# User-friendly race trajectory optimizer

# Default values
TRACK=""
METHOD=""
CONFIG="/app/config/full_config.yaml"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -t|--track)
            TRACK="$2"
            shift 2
            ;;
        -m|--method)
            METHOD="$2"
            shift 2
            ;;
        -c|--config)
            CONFIG="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  -t, --track TRACK      Track name (e.g., berlin_2018)"
            echo "  -m, --method METHOD    Optimization method:"
            echo "                         geometric, min_curv, shortest_path, min_time"
            echo "  -c, --config FILE      Config file path"
            echo "  -h, --help            Show this help"
            echo ""
            echo "Examples:"
            echo "  $0 -t berlin_2018 -m min_time"
            echo "  $0 -t modena_2019 -m geometric"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Build command
CMD="python /app/src/race_trajectory_optimizer.py"
if [ ! -z "$TRACK" ]; then
    CMD="$CMD --track $TRACK"
fi
if [ ! -z "$METHOD" ]; then
    CMD="$CMD --method $METHOD"
fi
CMD="$CMD --config $CONFIG"

# Run optimizer
echo "=== Race Trajectory Optimizer ==="
echo "Running: $CMD"
echo

docker run -it --rm \
    -v $(pwd)/src:/app/src \
    -v $(pwd)/config:/app/config \
    -v $(pwd)/inputs:/app/inputs \
    -v $(pwd)/outputs:/app/outputs \
    -v $(pwd)/global_racetrajectory_optimization:/app/global_racetrajectory_optimization \
    race-trajectory-optimizer-tumftm \
    $CMD

# Show results
echo
echo "Results saved to outputs/"
ls -la outputs/*_analysis.png outputs/*_trajectory.csv outputs/*_stats.json 2>/dev/null || true

# Try to open visualization
if command -v xdg-open &> /dev/null; then
    if [ ! -z "$TRACK" ]; then
        xdg-open "outputs/${TRACK}_analysis.png" 2>/dev/null || true
    else
        xdg-open outputs/*_analysis.png 2>/dev/null || true
    fi
fi
