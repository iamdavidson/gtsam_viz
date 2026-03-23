#include "GraphLayout.h"
#include <cmath>
#include <algorithm>

namespace gtsam_viz {

static float safelen(glm::vec2 v) {
    float l = glm::length(v);
    return l < 1e-4f ? 1e-4f : l;
}

void GraphLayout::step(FactorGraphState& state, int steps) {
    for (int i = 0; i < steps; ++i) applyForces(state);
}

void GraphLayout::resetVelocities(FactorGraphState& state) {
    for (auto& v : state.variables()) v.velocity = {0,0};
    for (auto& f : state.factors())   f.velocity = {0,0};
}

void GraphLayout::circularInit(FactorGraphState& state,
                               float cx, float cy, float radius) {
    auto& vars = state.variables();
    size_t n   = vars.size();
    for (size_t i = 0; i < n; ++i) {
        float angle   = 2.f * 3.14159265f * float(i) / float(n ? n : 1);
        vars[i].pos.x = cx + radius * std::cos(angle);
        vars[i].pos.y = cy + radius * std::sin(angle);
        vars[i].velocity = {0,0};
    }
    // Place factors at centroid of their variables, offset slightly
    for (auto& fn : state.factors()) {
        if (fn.keys.empty()) continue;
        glm::vec2 centroid{0,0};
        int count = 0;
        for (auto k : fn.keys) {
            for (auto& vn : vars) {
                if (vn.key == k) { centroid += vn.pos; ++count; break; }
            }
        }
        if (count) fn.pos = centroid / float(count);
        fn.velocity = {0,0};
    }
}

void GraphLayout::gridInit(FactorGraphState& state,
                           float cx, float cy, float spacing) {
    auto& vars = state.variables();
    size_t n   = vars.size();
    int cols   = std::max(1, (int)std::ceil(std::sqrt((float)n)));
    for (size_t i = 0; i < n; ++i) {
        vars[i].pos.x = cx + float(i % cols) * spacing;
        vars[i].pos.y = cy + float(i / cols) * spacing;
        vars[i].velocity = {0,0};
    }
    for (auto& fn : state.factors()) fn.velocity = {0,0};
}

float GraphLayout::kineticEnergy(const FactorGraphState& state) const {
    float e = 0;
    for (auto& v : state.variables()) e += glm::dot(v.velocity, v.velocity);
    for (auto& f : state.factors())   e += glm::dot(f.velocity, f.velocity);
    return e;
}

void GraphLayout::applyForces(FactorGraphState& state) {
    auto& vars    = state.variables();
    auto& factors = state.factors();
    float dt      = cfg_.timeStep;
    float k_rep   = cfg_.repulsion;
    float k_att   = cfg_.attraction;
    float damp    = cfg_.damping;
    float maxV    = cfg_.maxVelocity;

    // Build unified list of node positions for repulsion
    // Variables + factors (if separateFactorNodes)
    struct Node { glm::vec2* pos; glm::vec2* vel; bool pinned; };
    std::vector<Node> nodes;
    for (auto& v : vars)    nodes.push_back({&v.pos, &v.velocity, v.fixed});
    if (cfg_.separateFactorNodes)
        for (auto& f : factors) nodes.push_back({&f.pos, &f.velocity, false});

    // ── Repulsion between all pairs ───────────────────────────────────────────
    size_t N = nodes.size();
    std::vector<glm::vec2> forces(N, {0,0});

    for (size_t i = 0; i < N; ++i) {
        for (size_t j = i+1; j < N; ++j) {
            glm::vec2 d   = *nodes[i].pos - *nodes[j].pos;
            float     len = safelen(d);
            float     force = k_rep / (len * len);
            glm::vec2 dir = d / len;
            forces[i] += dir * force;
            forces[j] -= dir * force;
        }
    }

    // ── Attraction: factor–variable edges (spring) ────────────────────────────
    auto findVarIdx = [&](gtsam::Key k) -> int {
        for (size_t i = 0; i < vars.size(); ++i)
            if (vars[i].key == k) return (int)i;
        return -1;
    };

    for (size_t fi = 0; fi < factors.size(); ++fi) {
        size_t fNodeIdx = cfg_.separateFactorNodes ? (vars.size() + fi) : SIZE_MAX;
        for (auto k : factors[fi].keys) {
            int vi = findVarIdx(k);
            if (vi < 0) continue;

            if (cfg_.separateFactorNodes && fNodeIdx < N) {
                // factor node ↔ variable node
                glm::vec2 d   = *nodes[vi].pos - *nodes[fNodeIdx].pos;
                float     len = safelen(d);
                glm::vec2 f   = d * (k_att * len);
                forces[vi]        -= f;
                forces[fNodeIdx]  += f;
            } else {
                // No factor nodes: attract variable pairs that share a factor
                for (auto k2 : factors[fi].keys) {
                    int vi2 = findVarIdx(k2);
                    if (vi2 < 0 || vi2 == vi) continue;
                    glm::vec2 d = *nodes[vi2].pos - *nodes[vi].pos;
                    float len   = safelen(d);
                    glm::vec2 f = d * (k_att * len * 0.5f);
                    forces[vi]  += f;
                    forces[vi2] -= f;
                }
            }
        }
    }

    // ── Integrate ─────────────────────────────────────────────────────────────
    for (size_t i = 0; i < N; ++i) {
        if (nodes[i].pinned) continue;
        *nodes[i].vel  = (*nodes[i].vel + forces[i] * dt) * damp;
        float speed    = glm::length(*nodes[i].vel);
        if (speed > maxV) *nodes[i].vel = (*nodes[i].vel / speed) * maxV;
        *nodes[i].pos += *nodes[i].vel * dt;
    }
}

} // namespace gtsam_viz
