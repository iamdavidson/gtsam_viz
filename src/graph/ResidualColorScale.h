#pragma once
#include "FactorGraphState.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
#include <glm/glm.hpp>

namespace gtsam_viz {

struct ResidualStats {
    int    validCount = 0;
    int    staleCount = 0;
    double median = 0.0;
    double p95 = 1.0;
    double max = 0.0;
    double scale = 1.0;
};

inline bool isResidualValid(double err) {
    return std::isfinite(err) && err >= 0.0;
}

inline ResidualStats computeResidualStats(const std::vector<FactorNode>& factors,
                                          bool nonUnaryOnly = true) {
    std::vector<double> values;
    values.reserve(factors.size());
    ResidualStats stats;

    for (const auto& f : factors) {
        if (!f.errorFresh) ++stats.staleCount;
        if (nonUnaryOnly && f.keys.size() < 2) continue;
        if (!f.errorValid || !isResidualValid(f.error)) continue;
        values.push_back(f.error);
    }

    stats.validCount = static_cast<int>(values.size());
    if (values.empty()) return stats;

    std::sort(values.begin(), values.end());
    auto pct = [&](double q) {
        size_t idx = static_cast<size_t>(std::round(q * double(values.size() - 1)));
        return values[std::min(idx, values.size() - 1)];
    };

    stats.median = pct(0.50);
    stats.p95    = pct(0.95);
    stats.max    = values.back();
    stats.scale  = std::max({stats.p95, stats.median * 2.0, 1e-6});
    return stats;
}

inline glm::vec4 residualColor(double err, double scale, float multiplier = 1.f,
                               bool fresh = true, bool valid = true) {
    if (!fresh) return {0.55f, 0.55f, 0.58f, 0.65f};
    if (!valid || !isResidualValid(err)) return {0.75f, 0.75f, 0.78f, 0.45f};

    float denom = static_cast<float>(std::max(scale, 1e-6));
    float t = static_cast<float>(err) / denom;
    t = glm::clamp(t * std::max(0.001f, multiplier), 0.f, 1.f);

    constexpr glm::vec4 green{0.18f, 0.95f, 0.35f, 0.95f};
    constexpr glm::vec4 yellow{1.00f, 0.88f, 0.15f, 0.95f};
    constexpr glm::vec4 red{1.00f, 0.18f, 0.12f, 0.95f};

    if (t < 0.5f)
        return green + (yellow - green) * (t * 2.f);
    return yellow + (red - yellow) * ((t - 0.5f) * 2.f);
}

inline uint32_t residualColorU32(double err, double scale, float multiplier = 1.f,
                                 bool fresh = true, bool valid = true) {
    glm::vec4 c = residualColor(err, scale, multiplier, fresh, valid);
    auto toByte = [](float v) {
        return static_cast<uint32_t>(glm::clamp(v, 0.f, 1.f) * 255.f + 0.5f);
    };
    uint32_t r = toByte(c.r), g = toByte(c.g), b = toByte(c.b), a = toByte(c.a);
    return (a << 24) | (b << 16) | (g << 8) | r;
}

inline const char* factorErrorSourceLabel(FactorErrorSource src) {
    switch (src) {
    case FactorErrorSource::Gtsam: return "GTSAM";
    case FactorErrorSource::Ipc:   return "IPC";
    case FactorErrorSource::Stale: return "stale";
    default:                       return "-";
    }
}

} // namespace gtsam_viz
