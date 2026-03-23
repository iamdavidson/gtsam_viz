#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

#include <string>
#include <vector>
#include <unordered_map>
#include <optional>
#include <functional>
#include <memory>
#include <glm/glm.hpp>

namespace gtsam_viz {

// ─── Node / Variable metadata ─────────────────────────────────────────────────
enum class VariableType { Pose2, Pose3, Point2, Point3, Unknown };

struct VariableNode {
    gtsam::Key   key;
    std::string  label;       // e.g. "x1", "l2"
    VariableType type;
    glm::vec2    pos;         // layout position (pixels)
    bool         selected = false;
    bool         fixed    = false;  // pinned in layout
    glm::vec2    velocity = {0,0};  // force-layout velocity

    // Cached value representations
    glm::vec3    position3d{0};     // position in 3D space
    glm::mat4    transform{1};      // full pose as mat4
};

// ─── Factor metadata ──────────────────────────────────────────────────────────
enum class FactorType { Prior, Between, Projection, Custom };

struct FactorNode {
    size_t       index;       // index in NonlinearFactorGraph
    std::string  label;       // type name (short)
    FactorType   type;
    glm::vec2    pos;         // layout position (centroid of connected vars)
    bool         selected = false;
    double       error    = 0.0;
    std::vector<gtsam::Key> keys;  // copied from KeyVector
    glm::vec2    velocity = {0,0};
};

// ─── Optimization history entry ───────────────────────────────────────────────
struct OptimizationStep {
    int    iteration;
    double totalError;
    double lambda;           // for LM
};

// ─── Optimizer type ───────────────────────────────────────────────────────────
enum class OptimizerType { LevenbergMarquardt, DogLeg, GaussNewton, ISAM2 };

// ─── Main state class ─────────────────────────────────────────────────────────
class FactorGraphState {
public:
    using ChangeCallback = std::function<void()>;

    FactorGraphState();
    ~FactorGraphState() = default;

    // ── Graph modification ────────────────────────────────────────────────────
    void clear();
    void setGraph(gtsam::NonlinearFactorGraph graph, gtsam::Values initialValues);
    void addFactor(gtsam::NonlinearFactor::shared_ptr factor);
    void addValue(gtsam::Key key, const gtsam::Value& value);

    // ── Optimization ──────────────────────────────────────────────────────────
    void  setOptimizerType(OptimizerType type) { optimizerType_ = type; }
    OptimizerType getOptimizerType() const { return optimizerType_; }
    bool  optimize(int maxIterations = 100);
    bool  optimizeOneStep();
    void  resetValues();

    // ── Accessors ─────────────────────────────────────────────────────────────
    const gtsam::NonlinearFactorGraph& graph()       const { return graph_; }
    const gtsam::Values&               values()      const { return values_; }
    const gtsam::Values&               initialValues() const { return initialValues_; }

    std::vector<VariableNode>&         variables()         { return variables_; }
    const std::vector<VariableNode>&   variables()   const { return variables_; }
    std::vector<FactorNode>&           factors()           { return factors_; }
    const std::vector<FactorNode>&     factors()     const { return factors_; }

    double totalError()      const;
    double factorError(size_t factorIdx) const;

    // Covariance (requires marginals computation)
    bool                        computeMarginals();
    std::optional<gtsam::Matrix> marginalCovariance(gtsam::Key key) const;

    const std::vector<OptimizationStep>& history() const { return history_; }
    bool isOptimized() const { return optimized_; }

    // ── Change notifications ──────────────────────────────────────────────────
    void onChanged(ChangeCallback cb) { changeCallbacks_.push_back(std::move(cb)); }

    // ── Metadata rebuild ──────────────────────────────────────────────────────
    void rebuildMetadata();

    // 25002500 IPC / visual-only mutations (no GTSAM objects involved) 2500250025002500250025002500250025002500250025002500250025002500250025002500
    void clearVisualOnly();
    void clearFactorsVisual();
    void upsertVariable(VariableNode vn);
    void appendFactorVisual(FactorNode fn);
    void notifyChangedPublic() { notifyChanged(); }

    // ── ISAM2 ────────────────────────────────────────────────────────────────
    gtsam::ISAM2Params& isam2Params() { return isam2Params_; }

    // ── LM params ─────────────────────────────────────────────────────────────
    gtsam::LevenbergMarquardtParams& lmParams() { return lmParams_; }

private:
    void notifyChanged();
    void updateVariableValues();
    VariableType detectType(gtsam::Key key) const;
    FactorType   detectFactorType(const gtsam::NonlinearFactor::shared_ptr& f) const;
    void         extractPose(VariableNode& vn) const;

    gtsam::NonlinearFactorGraph    graph_;
    gtsam::Values                  values_;
    gtsam::Values                  initialValues_;

    std::vector<VariableNode>      variables_;
    std::vector<FactorNode>        factors_;

    OptimizerType                  optimizerType_ = OptimizerType::LevenbergMarquardt;
    gtsam::LevenbergMarquardtParams lmParams_;
    gtsam::DoglegParams            doglegParams_;
    gtsam::GaussNewtonParams       gnParams_;
    gtsam::ISAM2Params             isam2Params_;

    std::unique_ptr<gtsam::ISAM2>  isam2_;
    bool                           isam2Initialized_ = false;

    bool                           optimized_ = false;
    std::vector<OptimizationStep>  history_;

    std::unique_ptr<gtsam::Marginals> marginals_;
    bool                           marginalsValid_ = false;

    std::vector<ChangeCallback>    changeCallbacks_;
};

} // namespace gtsam_viz
