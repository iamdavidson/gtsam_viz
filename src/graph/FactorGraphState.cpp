#include "FactorGraphState.h"

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <typeinfo>
#include <sstream>
#include <stdexcept>

namespace gtsam_viz {

// ─── Helpers ─────────────────────────────────────────────────────────────────
static std::string keyLabel(gtsam::Key key) {
    try {
        gtsam::Symbol sym(key);
        std::ostringstream oss;
        oss << sym.chr() << sym.index();
        return oss.str();
    } catch (...) {
        return "k" + std::to_string(key);
    }
}

// ─── FactorGraphState ─────────────────────────────────────────────────────────
FactorGraphState::FactorGraphState() {
    lmParams_.setVerbosityLM("SILENT");
    lmParams_.setVerbosity("SILENT");
    lmParams_.maxIterations = 100;
}

void FactorGraphState::clear() {
    graph_            = gtsam::NonlinearFactorGraph{};
    values_           = gtsam::Values{};
    initialValues_    = gtsam::Values{};
    variables_.clear();
    factors_.clear();
    history_.clear();
    optimized_        = false;
    marginalsValid_   = false;
    marginals_.reset();
    isam2_.reset();
    isam2Initialized_ = false;
    notifyChanged();
}

void FactorGraphState::setGraph(gtsam::NonlinearFactorGraph graph,
                                gtsam::Values initialValues) {
    graph_         = std::move(graph);
    values_        = initialValues;
    initialValues_ = initialValues;
    history_.clear();
    optimized_      = false;
    marginalsValid_ = false;
    marginals_.reset();
    rebuildMetadata();
    notifyChanged();
}

void FactorGraphState::addFactor(gtsam::NonlinearFactor::shared_ptr factor) {
    graph_.push_back(factor);
    marginalsValid_ = false;
    rebuildMetadata();
    notifyChanged();
}

void FactorGraphState::addValue(gtsam::Key key, const gtsam::Value& value) {
    if (values_.exists(key)) {
        values_.update(key, value);
        initialValues_.update(key, value);
    } else {
        values_.insert(key, value);
        initialValues_.insert(key, value);
    }
    marginalsValid_ = false;
    rebuildMetadata();
    notifyChanged();
}

// ── Optimization ─────────────────────────────────────────────────────────────
bool FactorGraphState::optimize(int maxIterations) {
    if (graph_.empty() || values_.empty()) return false;
    history_.clear();
    try {
        switch (optimizerType_) {
        case OptimizerType::LevenbergMarquardt: {
            auto params = lmParams_;
            params.maxIterations = maxIterations;
            gtsam::LevenbergMarquardtOptimizer opt(graph_, values_, params);
            // Manually step to collect history
            gtsam::Values current = values_;
            for (int i = 0; i < maxIterations; ++i) {
                double errBefore = graph_.error(current);
                current = opt.optimizeSafely();
                double errAfter = graph_.error(current);
                history_.push_back({i, errAfter, opt.lambda()});
                if (std::abs(errBefore - errAfter) < params.relativeErrorTol * errBefore + 1e-9)
                    break;
            }
            values_ = current;
            break;
        }
        case OptimizerType::DogLeg: {
            gtsam::DoglegOptimizer opt(graph_, values_, doglegParams_);
            values_ = opt.optimize();
            history_.push_back({0, graph_.error(values_), 0.0});
            break;
        }
        case OptimizerType::GaussNewton: {
            gtsam::GaussNewtonOptimizer opt(graph_, values_, gnParams_);
            values_ = opt.optimize();
            history_.push_back({0, graph_.error(values_), 0.0});
            break;
        }
        case OptimizerType::ISAM2: {
            if (!isam2Initialized_) {
                isam2_ = std::make_unique<gtsam::ISAM2>(isam2Params_);
                isam2Initialized_ = true;
            }
            gtsam::ISAM2Result result = isam2_->update(graph_, values_);
            values_ = isam2_->calculateEstimate();
            history_.push_back({0, graph_.error(values_), 0.0});
            break;
        }
        }
        optimized_      = true;
        marginalsValid_ = false;
        marginals_.reset();
        updateVariableValues();
        notifyChanged();
        return true;
    } catch (const std::exception& e) {
        // Propagate as log
        return false;
    }
}

bool FactorGraphState::optimizeOneStep() {
    if (graph_.empty() || values_.empty()) return false;
    try {
        gtsam::LevenbergMarquardtOptimizer opt(graph_, values_, lmParams_);
        values_ = opt.optimizeSafely(); // one "safe" step
        int iter = history_.empty() ? 0 : (history_.back().iteration + 1);
        history_.push_back({iter, graph_.error(values_), opt.lambda()});
        optimized_      = true;
        marginalsValid_ = false;
        updateVariableValues();
        notifyChanged();
        return true;
    } catch (...) {
        return false;
    }
}

void FactorGraphState::resetValues() {
    values_         = initialValues_;
    optimized_      = false;
    marginalsValid_ = false;
    marginals_.reset();
    history_.clear();
    updateVariableValues();
    notifyChanged();
}

double FactorGraphState::totalError() const {
    if (values_.empty()) return 0.0;
    return graph_.error(values_);
}

double FactorGraphState::factorError(size_t idx) const {
    if (idx >= graph_.size() || values_.empty()) return 0.0;
    return graph_[idx]->error(values_);
}

bool FactorGraphState::computeMarginals() {
    if (!optimized_ || graph_.empty() || values_.empty()) return false;
    try {
        marginals_     = std::make_unique<gtsam::Marginals>(graph_, values_);
        marginalsValid_ = true;
        return true;
    } catch (...) {
        marginalsValid_ = false;
        return false;
    }
}

std::optional<gtsam::Matrix> FactorGraphState::marginalCovariance(gtsam::Key key) const {
    if (!marginalsValid_ || !marginals_) return std::nullopt;
    try {
        return marginals_->marginalCovariance(key);
    } catch (...) {
        return std::nullopt;
    }
}

// ── Metadata ─────────────────────────────────────────────────────────────────
void FactorGraphState::rebuildMetadata() {
    // ── Variables ────────────────────────────────────────────────────────────
    // Keep existing positions for continuity
    std::unordered_map<gtsam::Key, glm::vec2> existingPos;
    for (auto& v : variables_) existingPos[v.key] = v.pos;

    variables_.clear();
    for (const auto& kv : values_) {
        VariableNode vn;
        vn.key   = kv.key;
        vn.label = keyLabel(kv.key);
        vn.type  = detectType(kv.key);
        if (existingPos.count(kv.key))
            vn.pos = existingPos[kv.key];
        else {
            // Random scatter initial positions
            vn.pos.x = static_cast<float>(rand() % 600 + 100);
            vn.pos.y = static_cast<float>(rand() % 400 + 100);
        }
        variables_.push_back(std::move(vn));
    }

    // ── Factors ──────────────────────────────────────────────────────────────
    std::unordered_map<size_t, glm::vec2> existingFpos;
    for (auto& f : factors_) existingFpos[f.index] = f.pos;

    factors_.clear();
    for (size_t i = 0; i < graph_.size(); ++i) {
        auto& f = graph_[i];
        if (!f) continue;
        FactorNode fn;
        fn.index = i;
        auto kv = f->keys();
        fn.keys.assign(kv.begin(), kv.end());
        fn.type  = detectFactorType(f);
        fn.error = factorError(i);

        // Short type name
        std::string tname = typeid(*f).name();
        // Demangle partially
        auto pos = tname.rfind(':');
        fn.label = (pos != std::string::npos) ? tname.substr(pos+1, 16) : tname.substr(0, 16);

        if (existingFpos.count(i))
            fn.pos = existingFpos[i];
        else {
            fn.pos.x = static_cast<float>(rand() % 600 + 100);
            fn.pos.y = static_cast<float>(rand() % 400 + 100);
        }
        factors_.push_back(std::move(fn));
    }
    updateVariableValues();
}

void FactorGraphState::updateVariableValues() {
    for (auto& vn : variables_) {
        if (!values_.exists(vn.key)) continue;
        extractPose(vn);
    }
    // Recompute factor errors
    for (auto& fn : factors_) {
        fn.error = factorError(fn.index);
    }
}

VariableType FactorGraphState::detectType(gtsam::Key key) const {
    if (!values_.exists(key)) return VariableType::Unknown;
    try {
        values_.at<gtsam::Pose2>(key);  return VariableType::Pose2;
    } catch (...) {}
    try {
        values_.at<gtsam::Pose3>(key);  return VariableType::Pose3;
    } catch (...) {}
    try {
        values_.at<gtsam::Point2>(key); return VariableType::Point2;
    } catch (...) {}
    try {
        values_.at<gtsam::Point3>(key); return VariableType::Point3;
    } catch (...) {}
    return VariableType::Unknown;
}

FactorType FactorGraphState::detectFactorType(
        const gtsam::NonlinearFactor::shared_ptr& f) const {
    std::string tn = typeid(*f).name();
    if (tn.find("Prior")   != std::string::npos) return FactorType::Prior;
    if (tn.find("Between") != std::string::npos) return FactorType::Between;
    if (tn.find("Project") != std::string::npos) return FactorType::Projection;
    return FactorType::Custom;
}

void FactorGraphState::extractPose(VariableNode& vn) const {
    switch (vn.type) {
    case VariableType::Pose2: {
        auto p = values_.at<gtsam::Pose2>(vn.key);
        vn.position3d = {(float)p.x(), (float)p.y(), 0.f};
        vn.transform  = glm::mat4(1.f);
        vn.transform  = glm::translate(vn.transform, vn.position3d);
        vn.transform  = glm::rotate(vn.transform, (float)p.theta(),
                                    glm::vec3(0,0,1));
        break;
    }
    case VariableType::Pose3: {
        auto p = values_.at<gtsam::Pose3>(vn.key);
        auto t = p.translation();
        vn.position3d = {(float)t.x(), (float)t.y(), (float)t.z()};
        // Build mat4 from Rot3
        auto R = p.rotation().matrix();
        vn.transform = glm::mat4(
            (float)R(0,0),(float)R(1,0),(float)R(2,0),0,
            (float)R(0,1),(float)R(1,1),(float)R(2,1),0,
            (float)R(0,2),(float)R(1,2),(float)R(2,2),0,
            (float)t.x(), (float)t.y(), (float)t.z(), 1
        );
        break;
    }
    case VariableType::Point2: {
        auto p = values_.at<gtsam::Point2>(vn.key);
        vn.position3d = {(float)p.x(), (float)p.y(), 0.f};
        vn.transform  = glm::translate(glm::mat4(1.f), vn.position3d);
        break;
    }
    case VariableType::Point3: {
        auto p = values_.at<gtsam::Point3>(vn.key);
        vn.position3d = {(float)p.x(), (float)p.y(), (float)p.z()};
        vn.transform  = glm::translate(glm::mat4(1.f), vn.position3d);
        break;
    }
    default: break;
    }
}

void FactorGraphState::notifyChanged() {
    for (auto& cb : changeCallbacks_) cb();
}

// ── IPC visual-only mutations ─────────────────────────────────────────────────

void FactorGraphState::clearVisualOnly() {
    variables_.clear();
    factors_.clear();
    marginalsValid_ = false;
    marginals_.reset();
}

void FactorGraphState::clearFactorsVisual() {
    factors_.clear();
}

void FactorGraphState::upsertVariable(VariableNode vn) {
    for (auto& existing : variables_) {
        if (existing.key == vn.key) {
            vn.pos      = existing.pos;
            vn.selected = existing.selected;
            vn.velocity = existing.velocity;
            existing = std::move(vn);
            return;
        }
    }
    // New variable — derive a sensible 2D layout pos from 3D position
    vn.pos = { vn.position3d.x * 30.f + 400.f,
               vn.position3d.z * 30.f + 300.f };   // use X/Z as top-down
    variables_.push_back(std::move(vn));
}

void FactorGraphState::appendFactorVisual(FactorNode fn) {
    factors_.push_back(std::move(fn));
}

} // namespace gtsam_viz
