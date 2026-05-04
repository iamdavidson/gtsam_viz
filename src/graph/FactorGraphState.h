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
#include <array>
#include <unordered_map>
#include <optional>
#include <functional>
#include <memory>
#include <glm/glm.hpp>

namespace gtsam_viz {

// ─── Variable node ────────────────────────────────────────────────────────────
enum class VariableType { Pose2, Pose3, Point2, Point3, Unknown };

struct VariableNode {
    gtsam::Key   key;
    std::string  label;
    VariableType type;
    glm::vec2    pos      = {0, 0};
    bool         selected = false;
    bool         fixed    = false;
    glm::vec2    velocity = {0, 0};

    // 3D representation
    glm::vec3    position3d{0};
    glm::mat4    transform{1};

    // Optional per-variable position covariance (from IPC or GTSAM marginals)
    bool         has_covariance = false;
    float        covariance3d[9] = {};  // row-major 3×3
};

// ─── Factor node ──────────────────────────────────────────────────────────────
enum class FactorType { Prior, Between, Projection, Custom };

struct FactorNode {
    size_t       index;
    std::string  label;
    FactorType   type;
    glm::vec2    pos      = {0, 0};
    bool         selected = false;
    double       error    = 0.0;
    std::vector<gtsam::Key> keys;
    glm::vec2    velocity = {0, 0};
};

// ─── Point cloud ──────────────────────────────────────────────────────────────
struct PointCloud {
    std::vector<glm::vec3>          points;
    std::vector<glm::u8vec4>        colors;        // empty → use default_color
    glm::vec4                       default_color  = {1, 1, 1, 1};
    float                           point_size     = 3.f;
    std::string                     label;
};

// ─── Primitive ────────────────────────────────────────────────────────────────
// prim_type mirrors gviz_ipc::PrimType; we avoid including the IPC header here
// to keep the state layer free of wire-format details.
enum class PrimType : uint8_t {
    Line = 0, Arrow, Box, Sphere, Cone, Cylinder, CoordFrame
};

struct Primitive {
    PrimType     type;
    glm::vec4    color;
    float        data[16] = {};  // same layout as GVizPrimEntry::data
};

// ─── Optimization history ─────────────────────────────────────────────────────
struct OptimizationStep {
    int    iteration;
    double totalError;
    double lambda;
};

// ─── Optimizer type ───────────────────────────────────────────────────────────
enum class OptimizerType { LevenbergMarquardt, DogLeg, GaussNewton, ISAM2 };

// ─── Main state ───────────────────────────────────────────────────────────────
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
    void  setOptimizerType(OptimizerType t) { optimizerType_ = t; }
    OptimizerType getOptimizerType()  const { return optimizerType_; }
    bool  optimize(int maxIterations = 100);
    bool  optimizeOneStep();
    void  resetValues();

    // ── Accessors ─────────────────────────────────────────────────────────────
    const gtsam::NonlinearFactorGraph& graph()          const { return graph_; }
    const gtsam::Values&               values()         const { return values_; }
    const gtsam::Values&               initialValues()  const { return initialValues_; }

    std::vector<VariableNode>&         variables()             { return variables_; }
    const std::vector<VariableNode>&   variables()       const { return variables_; }
    std::vector<FactorNode>&           factors()               { return factors_; }
    const std::vector<FactorNode>&     factors()         const { return factors_; }

    // Point clouds
    const std::vector<PointCloud>&     pointClouds()     const { return point_clouds_; }
    void setPointClouds(std::vector<PointCloud> clouds);
    void clearPointClouds();

    // Primitives
    const std::vector<Primitive>&      primitives()      const { return primitives_; }
    void setPrimitives(std::vector<Primitive> prims);
    void clearPrimitives();

    // ── Errors ───────────────────────────────────────────────────────────────
    double totalError()            const;
    double factorError(size_t idx) const;

    // ── Covariance (in-process optimizer path) ────────────────────────────────
    bool                          computeMarginals();
    std::optional<gtsam::Matrix>  marginalCovariance(gtsam::Key key) const;

    const std::vector<OptimizationStep>& history()    const { return history_; }
    bool isOptimized()                               const { return optimized_; }

    // ── Change notifications ──────────────────────────────────────────────────
    void onChanged(ChangeCallback cb) { changeCallbacks_.push_back(std::move(cb)); }

    // ── Metadata rebuild ──────────────────────────────────────────────────────
    void rebuildMetadata();

    // ── IPC / visual-only mutations ───────────────────────────────────────────
    void clearVisualOnly();
    void clearFactorsVisual();
    void upsertVariable(VariableNode vn);
    void appendFactorVisual(FactorNode fn);
    void notifyChangedPublic() { notifyChanged(); }

    // ── ISAM2 / LM params ────────────────────────────────────────────────────
    gtsam::ISAM2Params&              isam2Params() { return isam2Params_; }
    gtsam::LevenbergMarquardtParams& lmParams()    { return lmParams_; }

private:
    void notifyChanged();
    void updateVariableValues();
    VariableType detectType(gtsam::Key key) const;
    FactorType   detectFactorType(const gtsam::NonlinearFactor::shared_ptr& f) const;
    void         extractPose(VariableNode& vn) const;

    gtsam::NonlinearFactorGraph     graph_;
    gtsam::Values                   values_;
    gtsam::Values                   initialValues_;

    std::vector<VariableNode>       variables_;
    std::vector<FactorNode>         factors_;
    std::vector<PointCloud>         point_clouds_;
    std::vector<Primitive>          primitives_;

    OptimizerType                   optimizerType_ = OptimizerType::LevenbergMarquardt;
    gtsam::LevenbergMarquardtParams lmParams_;
    gtsam::DoglegParams             doglegParams_;
    gtsam::GaussNewtonParams        gnParams_;
    gtsam::ISAM2Params              isam2Params_;

    std::unique_ptr<gtsam::ISAM2>   isam2_;
    bool                            isam2Initialized_ = false;

    bool                            optimized_      = false;
    std::vector<OptimizationStep>   history_;

    std::unique_ptr<gtsam::Marginals> marginals_;
    bool                            marginalsValid_ = false;

    std::vector<ChangeCallback>     changeCallbacks_;
};

} // namespace gtsam_viz
