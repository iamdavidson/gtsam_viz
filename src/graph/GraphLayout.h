#pragma once
#include "FactorGraphState.h"
#include <glm/glm.hpp>
#include <vector>

namespace gtsam_viz {

struct LayoutConfig {
    float repulsion    = 12000.f;  // node-node repulsion constant
    float attraction   = 0.04f;   // edge spring constant
    float damping      = 0.88f;   // velocity damping per step
    float nodeRadius   = 22.f;    // used for collision
    float factorRadius = 12.f;
    float timeStep     = 0.016f;  // dt in seconds
    float maxVelocity  = 150.f;
    float minEnergyStop= 0.5f;    // stop if kinetic energy below this
    bool  separateFactorNodes = true; // treat factor nodes as layout nodes too
};

class GraphLayout {
public:
    explicit GraphLayout(LayoutConfig cfg = {}) : cfg_(cfg) {}

    // Run N force-layout steps; modifies pos/velocity in place
    void step(FactorGraphState& state, int steps = 1);

    // Reset velocities (e.g. after graph change)
    void resetVelocities(FactorGraphState& state);

    // Place nodes in a deterministic circle/grid (useful starting point)
    void circularInit(FactorGraphState& state, float cx = 400, float cy = 300,
                      float radius = 250);
    void gridInit(FactorGraphState& state, float cx = 400, float cy = 300,
                  float spacing = 80);

    // Returns approximate kinetic energy (convergence indicator)
    float kineticEnergy(const FactorGraphState& state) const;

    bool   running = false;
    LayoutConfig& config() { return cfg_; }

private:
    void applyForces(FactorGraphState& state);

    LayoutConfig cfg_;
};

} // namespace gtsam_viz
