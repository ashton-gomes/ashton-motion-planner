#include "sampler.hpp"

CandidateSampler::CandidateSampler(const Eigen::Vector3d& lo, const Eigen::Vector3d& hi,
                                   unsigned int seed)
    : ws_min_(lo), ws_max_(hi), rng_(seed), uni01_(0.0, 1.0) {}

Eigen::Vector3d CandidateSampler::sampleWorkspacePoint() {
    return Eigen::Vector3d(
        ws_min_.x() + rollUniform() * (ws_max_.x() - ws_min_.x()),
        ws_min_.y() + rollUniform() * (ws_max_.y() - ws_min_.y()),
        ws_min_.z() + rollUniform() * (ws_max_.z() - ws_min_.z())
    );
}
