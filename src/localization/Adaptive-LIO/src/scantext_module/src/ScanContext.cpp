#include "scantext_module/ScanContext.hpp"

namespace scantext {

ScanContext::ScanContext(const SCParams& params) : params_(params) {}

ScanContext::SCDescriptor ScanContext::makeScanContext(const PointCloudType& scan) {
    int num_ring = params_.num_ring;
    int num_sector = params_.num_sector;
    double max_radius = params_.max_radius;
    double gap_ring = max_radius / num_ring;
    double gap_sector = 2.0 * M_PI / num_sector;

    SCDescriptor sc_desc = Eigen::MatrixXd::Zero(num_ring, num_sector);

    for (const auto& pt : scan.points) {
        float x = pt.x;
        float y = pt.y;
        float z = pt.z;

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            continue;

        float range = std::sqrt(x * x + y * y);
        if (range > max_radius)
            continue;

        float angle = std::atan2(y, x); // [-PI, PI]
        if (angle < 0) angle += 2.0 * M_PI; // [0, 2PI]

        int idx_ring = std::min(std::max((int)(range / gap_ring), 0), num_ring - 1);
        int idx_sector = std::min(std::max((int)(angle / gap_sector), 0), num_sector - 1);

        // ScanContext stores the maximum height in each bin
        // We add an offset to z to ensure positive values if needed, 
        // but typically just taking max z is fine. 
        // Some implementations use z + 2.0 to make it positive.
        if (sc_desc(idx_ring, idx_sector) == 0.0)
            sc_desc(idx_ring, idx_sector) = z;
        else
            sc_desc(idx_ring, idx_sector) = std::max(sc_desc(idx_ring, idx_sector), (double)z);
    }

    return sc_desc;
}

ScanContext::RingKey ScanContext::makeRingKey(const SCDescriptor& sc) {
    // RingKey is the mean of each row (ring)
    // Actually, original paper uses L0 norm (occupancy) or average intensity. 
    // Common implementation uses row averages of the height map.
    Eigen::VectorXd ring_key = sc.rowwise().mean();
    return ring_key;
}

ScanContext::SCDescriptor ScanContext::circshift(const SCDescriptor& sc, int shift) {
    int rows = sc.rows();
    int cols = sc.cols();
    SCDescriptor shifted_sc(rows, cols);
    
    // Handle negative shift
    if(shift < 0) shift = cols + shift;
    shift = shift % cols;

    if (shift == 0) return sc;

    shifted_sc.leftCols(cols - shift) = sc.rightCols(cols - shift);
    shifted_sc.rightCols(shift) = sc.leftCols(shift);

    return shifted_sc;
}

std::pair<double, int> ScanContext::distanceBtnScanContext(const SCDescriptor& sc1, const SCDescriptor& sc2) {
    int num_sector = params_.num_sector;
    int best_shift = 0;
    double min_dist = 1.0; // Distance is cosine distance, so 1.0 is max (irrelevant)

    // We search for the best shift to align sc2 to sc1
    // Cosine distance calculation
    // A simple column-wise difference metric is often used
    
    // Using simple sum of absolute differences or cosine distance per column?
    // The original paper suggests column-wise cosine distance.
    
    // Let's implement the standard approach:
    // For each shift, calculate the sum of errors.
    
    min_dist = std::numeric_limits<double>::max();

    for (int shift = 0; shift < num_sector; ++shift) {
        SCDescriptor sc2_shifted = circshift(sc2, shift);
        
        // Compute distance (e.g., L1 difference)
        // Or cosine distance.
        // Let's use column-wise cosine distance sum, normalized.
        
        double dist = 0.0;
        int valid_cols = 0;
        
        // A simple metric: Sum of L1 norms of columns, normalized?
        // Fast implementation: Element-wise difference
        // dist = (sc1 - sc2_shifted).norm() / sc1.norm(); // This is Frobenius norm based
        
        // Paper uses: 1 - mean( dot(c1, c2) / (|c1|*|c2|) )
        
        double sum_cos_sim = 0.0;
        for (int c = 0; c < sc1.cols(); ++c) {
            Eigen::VectorXd col1 = sc1.col(c);
            Eigen::VectorXd col2 = sc2_shifted.col(c);
            
            double norm1 = col1.norm();
            double norm2 = col2.norm();
            
            if (norm1 == 0 || norm2 == 0) {
                // If one is empty, similarity is 0? Or skip?
                // If both empty, similarity 1.
                if (norm1 == 0 && norm2 == 0) sum_cos_sim += 1.0;
                continue;
            }
            
            double cos_sim = col1.dot(col2) / (norm1 * norm2);
            sum_cos_sim += cos_sim;
        }
        
        double mean_cos_sim = sum_cos_sim / num_sector;
        double current_dist = 1.0 - mean_cos_sim;
        
        if (current_dist < min_dist) {
            min_dist = current_dist;
            best_shift = shift;
        }
    }

    return {min_dist, best_shift};
}

} // namespace scantext
