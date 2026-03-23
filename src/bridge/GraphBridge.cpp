#include "GraphBridge.h"
#include "../graph/FactorGraphState.h"
#include "../gui/panels/LogPanel.h"

#include <imgui.h>
#include <chrono>
#include <sstream>

namespace gtsam_viz {

// ─────────────────────────────────────── GraphBridge ─────────────────────────

GraphBridge& GraphBridge::instance() {
    static GraphBridge inst;
    return inst;
}

GraphBridge::GraphBridge() = default;

double GraphBridge::nowSec() const {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// ─── Backend API ──────────────────────────────────────────────────────────────

void GraphBridge::publish(const gtsam::NonlinearFactorGraph& graph,
                          const gtsam::Values& values,
                          PublishMode mode,
                          const std::string& label) {
    std::lock_guard<std::mutex> g(mtx_);

    if (hasNew_) ++stats_.dropCount;  // previous snapshot not yet consumed

    GraphSnapshot snap;
    snap.mode        = mode;
    snap.values      = values;
    snap.label       = label;
    snap.sequenceId  = ++seqCounter_;
    snap.timestampSec= nowSec();

    if (mode == PublishMode::Append) {
        // Merge into accumulator
        for (auto& f : graph)
            if (f) accumulated_.push_back(f);
        snap.graph = accumulated_;
    } else if (mode == PublishMode::ValuesOnly) {
        snap.graph = accumulated_; // keep existing graph
    } else {
        // Replace: update accumulator too
        accumulated_    = graph;
        accumulatedVals_= values;
        snap.graph      = graph;
    }

    pending_ = std::move(snap);
    hasNew_  = true;

    updateStats(snap.timestampSec);
    cv_.notify_one();
}

void GraphBridge::publishValues(const gtsam::Values& values,
                                const std::string& label) {
    publish({}, values, PublishMode::ValuesOnly, label);
}

void GraphBridge::appendFactor(gtsam::NonlinearFactor::shared_ptr factor) {
    std::lock_guard<std::mutex> g(mtx_);
    accumulated_.push_back(factor);

    GraphSnapshot snap;
    snap.mode        = PublishMode::Append;
    snap.graph       = accumulated_;
    snap.values      = accumulatedVals_;
    snap.sequenceId  = ++seqCounter_;
    snap.timestampSec= nowSec();

    pending_ = std::move(snap);
    hasNew_  = true;
    cv_.notify_one();
}

void GraphBridge::appendVariable(gtsam::Key key, const gtsam::Value& value) {
    std::lock_guard<std::mutex> g(mtx_);
    if (accumulatedVals_.exists(key))
        accumulatedVals_.update(key, value);
    else
        accumulatedVals_.insert(key, value);

    GraphSnapshot snap;
    snap.mode        = PublishMode::ValuesOnly;
    snap.graph       = accumulated_;
    snap.values      = accumulatedVals_;
    snap.sequenceId  = ++seqCounter_;
    snap.timestampSec= nowSec();

    pending_ = std::move(snap);
    hasNew_  = true;
    cv_.notify_one();
}

void GraphBridge::clear() {
    std::lock_guard<std::mutex> g(mtx_);
    accumulated_     = gtsam::NonlinearFactorGraph{};
    accumulatedVals_ = gtsam::Values{};
    pending_         = GraphSnapshot{};  // empty Replace
    pending_->mode   = PublishMode::Replace;
    hasNew_          = true;
    cv_.notify_one();
}

// ─── GUI API (Main Thread) ────────────────────────────────────────────────────

bool GraphBridge::poll(FactorGraphState& state) {
    double now = nowSec();
    if ((now - lastPollTime_) * 1000.0 < minUpdateIntervalMs_)
        return false;
    lastPollTime_ = now;

    std::optional<GraphSnapshot> snap;
    {
        std::lock_guard<std::mutex> g(mtx_);
        if (!hasNew_) return false;
        snap    = std::move(pending_);
        hasNew_ = false;
        pending_.reset();
    }
    if (!snap) return false;

    // Apply snapshot to FactorGraphState
    switch (snap->mode) {
    case PublishMode::Replace:
        state.setGraph(std::move(snap->graph), snap->values);
        break;

    case PublishMode::Append:
        // Re-set whole graph (accumulated is already complete in snapshot)
        state.setGraph(std::move(snap->graph), snap->values);
        break;

    case PublishMode::ValuesOnly:
        // Update values only; keep existing graph structure
        for (const auto& kv : snap->values) {
            state.addValue(kv.key, kv.value);
        }
        break;
    }

    // Log
    {
        std::ostringstream oss;
        oss << "[Bridge] seq=" << snap->sequenceId
            << " mode=" << (snap->mode == PublishMode::Replace   ? "Replace"
                          : snap->mode == PublishMode::Append    ? "Append"
                                                                 : "ValuesOnly");
        if (!snap->label.empty()) oss << " \"" << snap->label << "\"";
        oss << "  vars=" << state.variables().size()
            << " factors=" << state.factors().size();
        GVLOG_DEBUG(oss.str());
    }

    // Fire callbacks
    for (auto& cb : updateCallbacks_) cb(*snap);

    return true;
}

void GraphBridge::onUpdate(std::function<void(const GraphSnapshot&)> cb) {
    std::lock_guard<std::mutex> g(mtx_);
    updateCallbacks_.push_back(std::move(cb));
}

void GraphBridge::updateStats(double t) {
    ++stats_.publishCount;
    double dt = t - lastPublishTimeStat_;
    if (lastPublishTimeStat_ > 0.0 && dt > 0.0)
        stats_.avgPublishHz = stats_.avgPublishHz * 0.8 + (1.0 / dt) * 0.2;
    stats_.lastPublishTime  = t;
    lastPublishTimeStat_    = t;
}

// ─────────────────────────────────────── BridgePanel ─────────────────────────

BridgePanel::BridgePanel(GraphBridge& bridge) : bridge_(bridge) {}

void BridgePanel::draw() {
    auto& stats = bridge_.stats();

    // ── Connection indicator ──────────────────────────────────────────────
    bool connected = bridge_.isBackendConnected();
    ImGui::PushStyleColor(ImGuiCol_Text,
        connected ? ImVec4{0.2f,0.95f,0.4f,1.f} : ImVec4{0.6f,0.6f,0.65f,1.f});
    ImGui::Text(connected ? "⬤  Backend VERBUNDEN" : "⬤  Kein Backend");
    ImGui::PopStyleColor();

    ImGui::SameLine(200);
    ImGui::TextDisabled("seq: %llu  drops: %llu",
        (unsigned long long)stats.publishCount,
        (unsigned long long)stats.dropCount);

    // ── Hz sparkline ─────────────────────────────────────────────────────
    ImGui::Spacing();
    float hz = (float)stats.avgPublishHz;
    hzHistory_[hzIdx_] = hz;
    hzIdx_ = (hzIdx_ + 1) % HZ_HISTORY;
    ImGui::Text("Publish rate: %.1f Hz", hz);
    ImGui::PlotLines("##hz", hzHistory_, HZ_HISTORY, hzIdx_,
                     nullptr, 0.f, 120.f, {-1, 40});

    ImGui::Separator();

    // ── Controls ─────────────────────────────────────────────────────────
    ImGui::Checkbox("Updates pausieren", &pauseUpdates_);
    ImGui::SameLine();
    ImGui::Checkbox("Auto-Optimierung", &autoOptimize_);
    if (autoOptimize_) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        ImGui::InputInt("Iter##ao", &autoOptMaxIter_, 1, 5);
        autoOptMaxIter_ = std::max(1, std::min(500, autoOptMaxIter_));

        if (statePtr_) {
            ImGui::TextDisabled("→ Nach jedem Update: %d LM-Iterationen",
                                autoOptMaxIter_);
        }
    }

    ImGui::Separator();
    ImGui::SeparatorText("Publish-Modus Doku");
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4{0.08f,0.09f,0.11f,1.f});
    ImGui::BeginChild("##bridgedoc", {-1, 130}, true);
    ImGui::TextDisabled(
        "Replace   – Kompletter Graph-Austausch (Keyframe-Snapshot)\n"
        "Append    – Nur neue Faktoren/Variablen hinzufügen\n"
        "ValuesOnly– Nur Positions-Update, Struktur bleibt\n\n"
        "Backend-API:\n"
        "  SLAMPublisher pub;\n"
        "  pub.connect();\n"
        "  pub.publishReplace(graph, values, \"kf42\");\n"
        "  pub.publishValuesUpdate(isam.calculateEstimate());"
    );
    ImGui::EndChild();
    ImGui::PopStyleColor();

    // ── Manual test publish ───────────────────────────────────────────────
    ImGui::Spacing();
    if (ImGui::Button("🗑 Bridge leeren")) bridge_.clear();
    ImGui::SameLine();
    ImGui::SetNextItemWidth(180);
    ImGui::InputText("Label##bl", labelFilter_, sizeof(labelFilter_));
}

} // namespace gtsam_viz
