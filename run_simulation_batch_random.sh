#!/bin/bash

# Script to run CPFA simulation 30 times with random seeds and collect results
# Usage: ./run_simulation_batch_random.sh <config_file> <algorithm_mode>
# Example: ./run_simulation_batch_random.sh experiments/Clustered_CPFA_r10_10x10_4clusters_4resources.xml 1

if [ $# -ne 2 ]; then
    echo "Usage: $0 <config_file> <algorithm_mode>"
    echo "  config_file: Path to XML configuration file"
    echo "  algorithm_mode: 0 for baseline, 1 for enhanced"
    echo "Example: $0 experiments/Clustered_CPFA_r10_10x10_4clusters_4resources.xml 1"
    exit 1
fi

CONFIG_FILE="$1"
ALGORITHM_MODE="$2"
OUTPUT_FILE="simulation_results.csv"
TEMP_CONFIG="/tmp/temp_config_random_$$.xml"

# Check if config file exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Configuration file '$CONFIG_FILE' not found!"
    exit 1
fi

# Validate algorithm mode
if [ "$ALGORITHM_MODE" != "0" ] && [ "$ALGORITHM_MODE" != "1" ]; then
    echo "Error: Algorithm mode must be 0 (baseline) or 1 (enhanced)"
    exit 1
fi

# Create output file with header if it doesn't exist
if [ ! -f "$OUTPUT_FILE" ]; then
    echo "score,sim_time_seconds,random_seed,rejected_locations,cells_visited,algorithm_mode" > "$OUTPUT_FILE"
    echo "Created output file: $OUTPUT_FILE"
fi

echo "Starting batch simulation with random seeds..."
echo "Configuration file: $CONFIG_FILE"
echo "Algorithm mode: $ALGORITHM_MODE ($([ "$ALGORITHM_MODE" = "0" ] && echo "baseline" || echo "enhanced"))"
echo "Number of runs: 30"
echo "Results will be appended to: $OUTPUT_FILE"
echo ""

# Function to generate random seed
generate_random_seed() {
    # Generate a random number between 100000 and 999999
    echo $((RANDOM * RANDOM % 900000 + 100000))
}

# Function to update SearchAlgorithmMode and random seed in XML
update_config() {
    local input_file="$1"
    local output_file="$2"
    local mode="$3"
    local seed="$4"
    
    # Create a temporary config with the specified algorithm mode and seed
    sed -e "s/SearchAlgorithmMode=\"[01]\"/SearchAlgorithmMode=\"$mode\"/g" \
        -e "s/random_seed=\"[0-9]*\"/random_seed=\"$seed\"/g" \
        "$input_file" > "$output_file"
}

# Store all generated seeds for reporting
GENERATED_SEEDS=()

# Run simulation 30 times with random seeds
for i in {1..30}; do
    SEED=$(generate_random_seed)
    GENERATED_SEEDS+=($SEED)
    
    echo "Running simulation $i/30 with random seed $SEED..."
    
    # Update the algorithm mode and seed in the config file
    update_config "$CONFIG_FILE" "$TEMP_CONFIG" "$ALGORITHM_MODE" "$SEED"
    
    # Run the simulation and capture output
    # The simulation outputs: score, sim_time_seconds, random_seed, rejected_locations, cells_visited
    RESULT=$(argos3 -c "$TEMP_CONFIG" -z 2>/dev/null | grep -E "^[0-9.-]+, [0-9.-]+, [0-9]+$|^Total rejected random search locations: [0-9]+$|^Total cells visited: [0-9]+ / [0-9]+$")
    
    # Parse the output
    SCORE_LINE=$(echo "$RESULT" | grep -E "^[0-9.-]+, [0-9.-]+, [0-9]+$")
    REJECTION_LINE=$(echo "$RESULT" | grep "Total rejected random search locations:")
    CELLS_LINE=$(echo "$RESULT" | grep "Total cells visited:")
    
    if [ -n "$SCORE_LINE" ]; then
        # Extract score, sim_time, and seed from the first line
        SCORE=$(echo "$SCORE_LINE" | cut -d',' -f1 | tr -d ' ')
        SIM_TIME=$(echo "$SCORE_LINE" | cut -d',' -f2 | tr -d ' ')
        ACTUAL_SEED=$(echo "$SCORE_LINE" | cut -d',' -f3 | tr -d ' ')
        
        # Extract rejection count from the second line
        if [ -n "$REJECTION_LINE" ]; then
            REJECTED_COUNT=$(echo "$REJECTION_LINE" | grep -o '[0-9]\+$')
        else
            REJECTED_COUNT="0"
        fi
        
        # Extract cells visited count from the third line (format: "visited / total")
        if [ -n "$CELLS_LINE" ]; then
            CELLS_VISITED=$(echo "$CELLS_LINE" | grep -o '[0-9]\+ / [0-9]\+$')
        else
            CELLS_VISITED="0 / 0"
        fi
        
        # Append to output file
        echo "$SCORE,$SIM_TIME,$ACTUAL_SEED,$REJECTED_COUNT,\"$CELLS_VISITED\",$ALGORITHM_MODE" >> "$OUTPUT_FILE"
        
        echo "  Run $i: Score=$SCORE, Time=${SIM_TIME}s, Seed=$ACTUAL_SEED, Rejected=$REJECTED_COUNT, Cells=$CELLS_VISITED, Mode=$ALGORITHM_MODE"
    else
        echo "  Run $i: Failed to capture output properly"
        echo "  Raw output: $RESULT"
    fi
    
    # Small delay between runs
    sleep 1
done

# Clean up temporary file
rm -f "$TEMP_CONFIG"

echo ""
echo "Batch simulation completed!"
echo "Results saved to: $OUTPUT_FILE"
echo ""
echo "Generated random seeds used:"
echo "${GENERATED_SEEDS[*]}"
echo ""
echo "Summary (last 30 runs):"
tail -30 "$OUTPUT_FILE"

# Optional: Show statistics
echo ""
echo "Statistics for this batch (last 30 runs):"
echo "Average score: $(tail -30 "$OUTPUT_FILE" | grep -v "score,sim_time" | awk -F',' '{sum+=$1; count++} END {if(count>0) printf "%.2f", sum/count}')"
echo "Average simulation time: $(tail -30 "$OUTPUT_FILE" | grep -v "score,sim_time" | awk -F',' '{sum+=$2; count++} END {if(count>0) printf "%.2f", sum/count}')s"
echo "Total rejected locations: $(tail -30 "$OUTPUT_FILE" | grep -v "score,sim_time" | awk -F',' '{sum+=$4; count++} END {printf "%d", sum}')"
echo "Average cells visited: $(tail -30 "$OUTPUT_FILE" | grep -v "score,sim_time" | awk -F',' '{gsub(/\"/, "", $5); split($5, a, " / "); sum+=a[1]; count++} END {if(count>0) printf "%.2f", sum/count; else printf "0"}')"
echo "Min score: $(tail -30 "$OUTPUT_FILE" | grep -v "score,sim_time" | awk -F',' 'BEGIN{min=999999} {if($1<min) min=$1} END {printf "%.2f", min}')"
echo "Max score: $(tail -30 "$OUTPUT_FILE" | grep -v "score,sim_time" | awk -F',' 'BEGIN{max=-1} {if($1>max) max=$1} END {printf "%.2f", max}')"