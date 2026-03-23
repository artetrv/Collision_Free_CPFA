# CPFA Congestion-Aware Swarm Optimization Project
Author: Samantha Ocaña  
Platform: ARGoS + C++  
Objective: Reduce swarm collisions while maintaining or improving collection efficiency.

---

# 1. PROJECT OVERVIEW

This project modifies the baseline Central Place Foraging Algorithm (CPFA) in ARGoS to:

1. Detect congestion events.
2. Make a decision when congested:
   - Use Site Fidelity (SF), OR
   - Use Restricted Random Search (RS) in a quadrant away from congestion.
3. Compare performance against the baseline CPFA.

Primary focus:
- Collision mitigation
- Congestion-aware redirection
- Maintaining or improving resource collection performance

---

# 2. BASELINE BEHAVIOR (CONTROL)

Baseline CPFA logic:

SEARCHING → FOUND RESOURCE → RETURNING → DROP → DEPARTING → SEARCHING

After dropping resource:
- May use site fidelity (return near previous success)
- Otherwise random search

No congestion-aware logic in baseline.

---

# 3. MODIFIED LOGIC (CONGESTION-AWARE CPFA)

## 3.1 Congestion Detection

Congestion is triggered when:
- Collision count exceeds threshold
- OR congestion window ratio exceeds threshold

Upon congestion:

1. Robot drops resource.
2. Robot transitions to DEPARTING state.
3. Decision is made:

if (updateFidelity && sfValid && poissonCDF_sFollowRate > r2)
→ Use Site Fidelity
else
→ Use Restricted Random Search

---

## 3.2 Site Fidelity (SF) Path

When SF selected:
- SetTarget(SiteFidelityPosition)
- isInformed = true
- isUsingSiteFidelity = true
- hasRestrictedZone = false

Robot reuses baseline DEPARTING behavior.

---

## 3.3 Restricted Random Search (RS) Path

When RS selected:
- Create restricted quadrant away from congestion
- SetTarget(random location inside restricted zone)
- hasRestrictedZone = true

Includes:
- Near-wall buffer logic
- Anti-freeze mechanics
- Tolerance-based arrival checks

---

# 4. STATES INVOLVED

- SEARCHING
- RETURNING
- DEPARTING
- CONGESTED (event trigger)

No new major state introduced.
Congestion modifies transition logic only.

---

# 5. METRICS TO COLLECT

## 5.1 Performance Metrics

- Total resources collected
- Time to collect N resources
- Score per time
- Completion rate

## 5.2 Congestion Metrics

- numCongestionDrops
- numCongestedSF
- numCongestedRestricted
- Total collisions per robot
- Collisions per timestep

## 5.3 Efficiency Metrics

- Travel distance
- Idle time
- Cluster saturation time

---

# 6. EXPERIMENTAL MATRIX

Robot Counts:
- 16
- 24
- 32
- 40
- 48

Environment Types:
- Clustered food
- Random food

Strategies:
- Baseline CPFA
- Congestion-aware CPFA

Runs per condition:
- Minimum 30 runs

---

# 7. DATA OUTPUT STRUCTURE

Each experiment run should log:

robot_count,
food_distribution,
strategy,
resources_collected,
collisions_total,
numCongestedSF,
numCongestedRestricted,
time_to_completion

Save as CSV or structured text.

---

# 8. ANALYSIS PLAN

## 8.1 Box Plots

Compare:
- Collisions per robot
- Score per time

Look for:
- Long tails (variance)
- Median shifts
- Outliers

## 8.2 Statistical Comparison

Use:
- Median comparison
- Mann-Whitney U test
- Effect size

---

# 9. CLEAN LOGGING STRATEGY

Remove:
- Non-congestion DEPARTING logs
- Excess state transition spam

Keep:
- CONGESTED → SF
- CONGESTED → RS
- Final summary per run

---

# 10. OBJECTIVE

Primary Objective:
Reduce collisions without degrading collection efficiency.

Secondary Objective:
Demonstrate statistical improvement over baseline.

---

# 11. NEXT IMPLEMENTATION STEPS

1. Verify congestion detection thresholds.
2. Confirm SF vs RS branching logic.
3. Add counters for:
   - numCongestedSF
   - numCongestedRestricted
4. Validate no infinite loops in restricted zone selection.
5. Run 16 robot experiment (baseline vs modified).
6. Scale to 24, 32, 40, 48.
7. Export structured results.
8. Generate box plots.
9. Perform statistical comparison.
10. Prepare paper figures.

---

# 12. PAPER CONTRIBUTION STATEMENT

This work introduces a congestion-aware redirection mechanism in CPFA that dynamically chooses between site fidelity and spatially restricted exploration to mitigate swarm clustering and collision amplification in high-density foraging scenarios.

---

END OF MASTER PLAN