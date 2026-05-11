#include "InspectorPanel.h"
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <boost/shared_ptr.hpp>
#include <sstream>
#include <iomanip>
#include <imgui.h>
#include "../../graph/ResidualColorScale.h"

namespace gtsam_viz {

static std::string fmtDouble(double v, int prec=5) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(prec) << v;
    return oss.str();
}

InspectorPanel::InspectorPanel(FactorGraphState& state) : state_(state) {}

void InspectorPanel::draw(std::optional<gtsam::Key>& selVar,
                          std::optional<size_t>&     selFactor) {
    if (!selVar && !selFactor) {
        ImGui::TextDisabled("(nothing selected)");
        ImGui::Spacing();
        ImGui::TextDisabled("Click a node in the graph view to inspect it.");
        return;
    }

    if (selVar) {
        ImGui::TextColored({0.4f,0.8f,1.f,1.f}, "⬤ Variable");
        ImGui::SameLine();
        // Find var metadata
        for (auto& vn : state_.variables()) {
            if (vn.key == *selVar) {
                ImGui::Text("%s", vn.label.c_str());
                break;
            }
        }
        ImGui::Separator();
        drawVariableInspector(*selVar);
    }

    if (selFactor) {
        ImGui::TextColored({1.f,0.75f,0.2f,1.f}, "■ Factor");
        ImGui::SameLine();
        ImGui::Text("#%zu", *selFactor);
        ImGui::Separator();
        drawFactorInspector(*selFactor);
    }
}

void InspectorPanel::drawVariableInspector(gtsam::Key key) {
    // Key symbol info
    try {
        gtsam::Symbol sym(key);
        ImGui::Text("Symbol:  %c%llu", sym.chr(), (unsigned long long)sym.index());
    } catch (...) {
        ImGui::Text("Key: %llu", (unsigned long long)key);
    }

    // Find variable node
    VariableType vtype = VariableType::Unknown;
    for (auto& vn : state_.variables())
        if (vn.key == key) { vtype = vn.type; break; }

    const char* typeNames[] = {"Pose2","Pose3","Point2","Point3","Unknown"};
    ImGui::Text("Type:    %s", typeNames[(int)vtype]);
    ImGui::Spacing();

    // Values
    if (!state_.values().exists(key)) {
        ImGui::TextColored({1,0.4f,0.4f,1}, "No value in Values");
        return;
    }

    ImGui::SeparatorText("Current Value");
    switch (vtype) {
    case VariableType::Pose2:  drawPose2Values(key); break;
    case VariableType::Pose3:  drawPose3Values(key); break;
    case VariableType::Point2: {
        auto p = state_.values().at<gtsam::Point2>(key);
        ImGui::Text("x: %s", fmtDouble(p.x()).c_str());
        ImGui::Text("y: %s", fmtDouble(p.y()).c_str());
        break;
    }
    case VariableType::Point3: {
        auto p = state_.values().at<gtsam::Point3>(key);
        ImGui::Text("x: %s", fmtDouble(p.x()).c_str());
        ImGui::Text("y: %s", fmtDouble(p.y()).c_str());
        ImGui::Text("z: %s", fmtDouble(p.z()).c_str());
        break;
    }
    default:
        ImGui::TextDisabled("(unknown type – cannot display values)");
    }

    // Covariance
    ImGui::Spacing();
    ImGui::SeparatorText("Marginal Covariance");
    if (!state_.isOptimized()) {
        ImGui::TextDisabled("Run optimizer first.");
    } else {
        if (!marginalsComputed_) {
            if (ImGui::Button("Compute Marginals")) {
                marginalsComputed_ = state_.computeMarginals();
            }
        }
        if (marginalsComputed_) drawCovariance(key);
    }
}

void InspectorPanel::drawPose2Values(gtsam::Key key) {
    auto p = state_.values().at<gtsam::Pose2>(key);
    ImGui::Text("x:     %s m",   fmtDouble(p.x()).c_str());
    ImGui::Text("y:     %s m",   fmtDouble(p.y()).c_str());
    ImGui::Text("theta: %s rad", fmtDouble(p.theta()).c_str());
    ImGui::Text("       %s deg", fmtDouble(p.theta()*180.0/M_PI, 2).c_str());

    // Also show initial value delta
    if (state_.initialValues().exists(key)) {
        try {
            auto pi = state_.initialValues().at<gtsam::Pose2>(key);
            ImGui::Spacing();
            ImGui::SeparatorText("Δ from Initial");
            ImGui::Text("Δx:     %s", fmtDouble(p.x()-pi.x()).c_str());
            ImGui::Text("Δy:     %s", fmtDouble(p.y()-pi.y()).c_str());
            ImGui::Text("Δtheta: %s", fmtDouble(p.theta()-pi.theta()).c_str());
        } catch (...) {}
    }
}

void InspectorPanel::drawPose3Values(gtsam::Key key) {
    auto p  = state_.values().at<gtsam::Pose3>(key);
    auto t  = p.translation();
    auto rq = p.rotation().toQuaternion();
    ImGui::Text("x: %s", fmtDouble(t.x()).c_str());
    ImGui::Text("y: %s", fmtDouble(t.y()).c_str());
    ImGui::Text("z: %s", fmtDouble(t.z()).c_str());
    ImGui::Spacing();
    ImGui::Text("Quaternion (w,x,y,z):");
    ImGui::Text("  %.4f  %.4f  %.4f  %.4f",
                rq.w(), rq.x(), rq.y(), rq.z());
    auto rpy = p.rotation().rpy();
    ImGui::Text("Roll:  %s deg", fmtDouble(rpy(0)*180/M_PI,2).c_str());
    ImGui::Text("Pitch: %s deg", fmtDouble(rpy(1)*180/M_PI,2).c_str());
    ImGui::Text("Yaw:   %s deg", fmtDouble(rpy(2)*180/M_PI,2).c_str());
}

void InspectorPanel::drawCovariance(gtsam::Key key) {
    auto cov = state_.marginalCovariance(key);
    if (!cov) {
        ImGui::TextColored({1,0.4f,0.4f,1}, "Could not compute (singular?)");
        return;
    }
    drawMatrix(*cov, "Σ");

    // Trace (total uncertainty)
    double tr = cov->trace();
    ImGui::Text("trace(Σ) = %s", fmtDouble(tr).c_str());
}

void InspectorPanel::drawMatrix(const gtsam::Matrix& M, const char* label) {
    ImGui::Text("%s (%dx%d):", label, (int)M.rows(), (int)M.cols());
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f,0.13f,0.16f,1));
    float rowH = ImGui::GetTextLineHeightWithSpacing();
    ImVec2 sz  = {0.f, rowH * float(M.rows()) + 6};
    ImGui::BeginChild((std::string("##mat")+label).c_str(), sz, true);
    for (int r=0; r<(int)M.rows(); ++r) {
        for (int c=0; c<(int)M.cols(); ++c) {
            if (c>0) ImGui::SameLine(float(c)*70);
            ImGui::Text("%7.4f", M(r,c));
        }
    }
    ImGui::EndChild();
    ImGui::PopStyleColor();
}

void InspectorPanel::drawFactorInspector(size_t idx) {
    if (idx >= state_.factors().size()) {
        ImGui::TextDisabled("Factor index out of range");
        return;
    }
    auto& fn = state_.factors()[idx];

    ImGui::Text("Type:  %s", fn.label.c_str());
    ImGui::Text("Error: %s", fn.errorValid ? fmtDouble(fn.error).c_str() : "invalid");
    ImGui::Text("Source: %s%s", factorErrorSourceLabel(fn.errorSource),
                fn.errorFresh ? "" : " (stale)");
    ImGui::Spacing();
    ImGui::SeparatorText("Connected Variables");
    for (auto k : fn.keys) {
        try {
            gtsam::Symbol s(k);
            ImGui::BulletText("%c%llu", s.chr(), (unsigned long long)s.index());
        } catch (...) {
            ImGui::BulletText("key=%llu", (unsigned long long)k);
        }
    }

    // Linearized error vector (whitened residual)
    if (!state_.values().empty() && idx < state_.graph().size()) {
        try {
            auto& graph = state_.graph();
            if (idx < graph.size() && graph[idx]) {
                auto gf = graph[idx]->linearize(state_.values());
                // Access the linearized RHS (b-vector) via JacobianFactor
                auto jf = boost::dynamic_pointer_cast<gtsam::JacobianFactor>(gf);
                if (jf) {
                    const gtsam::Vector& b = jf->getb();
                    ImGui::Spacing();
                    ImGui::SeparatorText("Linearised RHS (b)");
                    ImGui::Text("dim = %d", (int)b.size());
                    for (int i = 0; i < (int)b.size(); ++i)
                        ImGui::Text("  [%d]: %s", i, fmtDouble(b(i)).c_str());
                }
            }
        } catch (...) {}
    }
}

} // namespace gtsam_viz
