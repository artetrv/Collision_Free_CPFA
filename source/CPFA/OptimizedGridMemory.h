#ifndef OPTIMIZED_GRID_MEMORY_H
#define OPTIMIZED_GRID_MEMORY_H

#include <set>
#include <vector>
#include <stdexcept>
#include <chrono>

/**
 * @struct GridMemoryMetrics
 * @brief Performance metrics for grid memory operations.
 */
struct GridMemoryMetrics
{
    // Query metrics
    size_t query_count = 0;
    long long query_total_microseconds = 0;
    double query_avg_latency = 0.0;

    // Update metrics
    size_t update_count = 0;
    long long update_total_microseconds = 0;
    double update_avg_latency = 0.0;
};

/**
 * @class OptimizedGridMemory
 * @brief Self-balancing grid memory system using std::set for O(log n) visit count updates.
 *
 * This class maintains a sorted order of grid cells by their visit frequencies in real-time.
 * It uses two complementary data structures:
 * - A std::set<std::pair<int, int>> for efficiently finding the least-visited cells
 * - A std::vector<int> for O(1) lookup of a cell's current visit count
 *
 * Time Complexity:
 * - get_least_visited_cell():  O(1)
 * - update_visit(cell_ID):     O(log n)
 * - initialization:            O(n log n)
 *
 * Space Complexity: O(n)
 */
class OptimizedGridMemory
{
public:
    /**
     * @brief Constructor for OptimizedGridMemory
     */
    OptimizedGridMemory() = default;

    /**
     * @brief Initialize the grid with N cells, all starting with visit count 0.
     *
     * @param num_cells The total number of grid cells to track.
     *
     * Time Complexity: O(n log n) where n = num_cells
     */
    void initialize(size_t num_cells)
    {
        // Clear any existing data
        visit_queue.clear();
        current_counts.clear();

        // Reserve space for efficiency
        current_counts.resize(num_cells, 0);

        // Insert all cells with visit count 0
        // Pair format: (visit_count, cell_ID)
        for (size_t cell_id = 0; cell_id < num_cells; ++cell_id)
        {
            visit_queue.insert({0, static_cast<int>(cell_id)});
        }
    }

    /**
     * @brief Get the cell ID of the least-visited cell.
     *
     * @return The cell_ID of the cell with the minimum visit count.
     *         If there are ties, returns the smallest cell_ID among tied cells.
     *
     * @throws std::runtime_error if the grid is empty.
     *
     * Time Complexity: O(1)
     */
    int get_least_visited_cell() const
    {
        // Start timer for query latency measurement
        auto query_start = std::chrono::high_resolution_clock::now();

        if (visit_queue.empty())
        {
            throw std::runtime_error("OptimizedGridMemory: visit_queue is empty. "
                                   "Ensure initialize() was called.");
        }

        // The set is sorted by visit_count (first element of pair)
        // The least-visited cell is at the beginning
        int result = visit_queue.begin()->second;

        // End timer and record latency
        auto query_end = std::chrono::high_resolution_clock::now();
        long long query_duration = std::chrono::duration_cast<std::chrono::microseconds>(query_end - query_start).count();
        
        // Update metrics (note: modifying mutable member in const function for metrics only)
        metrics.query_count++;
        metrics.query_total_microseconds += query_duration;

        return result;
    }

    /**
     * @brief Increment the visit count for a specific cell.
     *
     * This function:
     * 1. Retrieves the current visit count for the cell
     * 2. Removes the old (visit_count, cell_ID) pair from the set
     * 3. Increments the visit count
     * 4. Inserts the new (visit_count, cell_ID) pair into the set
     *
     * @param cell_id The ID of the cell to update.
     *
     * @throws std::out_of_range if cell_id is invalid.
     *
     * Time Complexity: O(log n)
     */
    void update_visit(int cell_id)
    {
        // Start timer for update latency measurement
        auto update_start = std::chrono::high_resolution_clock::now();

        // Bounds check
        if (cell_id < 0 || cell_id >= static_cast<int>(current_counts.size()))
        {
            throw std::out_of_range("OptimizedGridMemory: cell_id out of range. "
                                   "Valid range: [0, " + 
                                   std::to_string(current_counts.size() - 1) + "]");
        }

        // Step 1: Look up the old count
        int old_count = current_counts[cell_id];

        // Step 2: Erase the old pair from the set (O(log n))
        visit_queue.erase({old_count, cell_id});

        // Step 3: Increment the count
        int new_count = old_count + 1;

        // Step 4: Update the vector (O(1))
        current_counts[cell_id] = new_count;

        // Step 5: Insert the new pair into the set (O(log n))
        visit_queue.insert({new_count, cell_id});

        // End timer and record latency
        auto update_end = std::chrono::high_resolution_clock::now();
        long long update_duration = std::chrono::duration_cast<std::chrono::microseconds>(update_end - update_start).count();
        
        // Update metrics
        metrics.update_count++;
        metrics.update_total_microseconds += update_duration;
    }

    /**
     * @brief Get the current visit count of a specific cell.
     *
     * @param cell_id The ID of the cell to query.
     *
     * @return The visit count of the cell.
     *
     * @throws std::out_of_range if cell_id is invalid.
     *
     * Time Complexity: O(1)
     */
    int get_visit_count(int cell_id) const
    {
        if (cell_id < 0 || cell_id >= static_cast<int>(current_counts.size()))
        {
            throw std::out_of_range("OptimizedGridMemory: cell_id out of range. "
                                   "Valid range: [0, " + 
                                   std::to_string(current_counts.size() - 1) + "]");
        }
        return current_counts[cell_id];
    }

    /**
     * @brief Get the total number of cells in the grid.
     *
     * @return The number of cells.
     *
     * Time Complexity: O(1)
     */
    size_t get_num_cells() const
    {
        return current_counts.size();
    }

    /**
     * @brief Get the size of the visit queue (should equal num_cells).
     *
     * @return The number of entries in the visit queue.
     *
     * Time Complexity: O(1)
     */
    size_t get_queue_size() const
    {
        return visit_queue.size();
    }

    /**
     * @brief Reset all visit counts to 0.
     *
     * This clears the grid and reinitializes all counts.
     *
     * Time Complexity: O(n log n)
     */
    void reset()
    {
        size_t num_cells = current_counts.size();
        visit_queue.clear();
        std::fill(current_counts.begin(), current_counts.end(), 0);

        for (size_t cell_id = 0; cell_id < num_cells; ++cell_id)
        {
            visit_queue.insert({0, static_cast<int>(cell_id)});
        }
    }

    /**
     * @brief Get the current performance metrics.
     *
     * @return GridMemoryMetrics struct with query and update latency information.
     */
    GridMemoryMetrics get_metrics() const
    {
        GridMemoryMetrics result = metrics;
        
        // Calculate average latencies
        if (result.query_count > 0)
        {
            result.query_avg_latency = static_cast<double>(result.query_total_microseconds) / result.query_count;
        }
        
        if (result.update_count > 0)
        {
            result.update_avg_latency = static_cast<double>(result.update_total_microseconds) / result.update_count;
        }
        
        return result;
    }

    /**
     * @brief Reset performance metrics to zero.
     */
    void reset_metrics()
    {
        metrics.query_count = 0;
        metrics.query_total_microseconds = 0;
        metrics.query_avg_latency = 0.0;
        
        metrics.update_count = 0;
        metrics.update_total_microseconds = 0;
        metrics.update_avg_latency = 0.0;
    }

private:
    /**
     * @brief Priority queue of cells sorted by visit count (ascending).
     *
     * Each pair is (visit_count, cell_ID).
     * std::set automatically maintains sorted order by the first element of the pair.
     * In case of ties (same visit_count), cells are ordered by cell_ID.
     */
    std::set<std::pair<int, int>> visit_queue;

    /**
     * @brief Lookup table for O(1) visit count retrieval.
     *
     * Index: cell_ID
     * Value: current visit count for that cell
     */
    std::vector<int> current_counts;

    /**
     * @brief Performance metrics for grid operations.
     * Mutable to allow metrics tracking in const functions.
     */
    mutable GridMemoryMetrics metrics;
};

#endif /* OPTIMIZED_GRID_MEMORY_H */
