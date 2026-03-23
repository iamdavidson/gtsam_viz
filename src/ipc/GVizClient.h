#pragma once
/**
 * GVizClient.h  –  Header-only SLAM → GTSAMViz IPC client
 * =========================================================
 *
 * Kopiere NUR diese Datei + GVizProtocol.h in dein Projekt.
 * Keine weiteren Abhängigkeiten außer GTSAM + POSIX (Linux/macOS).
 *
 * BENUTZUNG
 * ─────────
 *   #include "GVizClient.h"
 *
 *   GVizClient viz;
 *   viz.connect();                                    // non-blocking, kein abort wenn GUI aus
 *
 *   // Nach jedem Keyframe:
 *   viz.publish(graph, values, "kf42");               // Replace-Snapshot
 *
 *   // Nur Positionen updaten (nach Optimierung):
 *   viz.publishValuesOnly(optimized_values);
 *
 *   // Inkrementell Kante hinzufügen:
 *   viz.appendEdge(key_from, key_to, GVizClient::Between, 0.0f);
 *
 *   viz.clear();        // Graph leeren
 *   viz.disconnect();   // Verbindung schließen
 *
 * THREAD-SICHERHEIT
 * ──────────────────
 *   Alle Methoden sind thread-safe — ein einzelner Mutex schützt den Socket.
 *   publish() blockiert maximal ~1 ms (Kernel-Buffer, LO).
 *
 * NULL-OVERHEAD WENN GUI NICHT LÄUFT
 * ────────────────────────────────────
 *   connect() schlägt lautlos fehl wenn kein Socket vorhanden.
 *   Alle publish*()-Calls prüfen zuerst connected_ und returnen sofort.
 *
 * VERBINDUNGSVERLUST
 *   Wenn die GUI abstürzt, erkennt GVizClient den broken-pipe und setzt
 *   connected_ = false. Der nächste publish()-Call versucht reconnect().
 */

#include "GVizProtocol.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

class GVizClient {
public:

    using FactorType = gviz_ipc::FactorType;
    static constexpr FactorType Prior      = FactorType::Prior;
    static constexpr FactorType Between    = FactorType::Between;
    static constexpr FactorType Projection = FactorType::Projection;
    static constexpr FactorType Custom     = FactorType::Custom;

    GVizClient() = default;
    ~GVizClient() { disconnect(); }

    // ── Verbindung ────────────────────────────────────────────────────────────

    /// Verbindet zum laufenden gtsam_viz-Prozess.
    /// Gibt true zurück wenn erfolgreich, false wenn GUI nicht läuft (kein Absturz).
    bool connect(const char* socket_path = gviz_ipc::SOCKET_PATH) {
        std::lock_guard<std::mutex> g(mtx_);
        if (fd_ >= 0) return true;

        socketPath_ = socket_path;
        fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd_ < 0) return false;

        struct sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, socket_path, sizeof(addr.sun_path)-1);

        if (::connect(fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            ::close(fd_); fd_ = -1;
            return false;
        }
        seq_ = 0;
        return true;
    }

    void disconnect() {
        std::lock_guard<std::mutex> g(mtx_);
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    bool isConnected() const { std::lock_guard<std::mutex> g(mtx_); return fd_ >= 0; }

    // ── Haupt-API ─────────────────────────────────────────────────────────────

    /**
     * Kompletten Graphen + Values publishen.
     * Typ = Replace: GUI ersetzt alles was sie bisher hat.
     *
     * @param graph   Dein NonlinearFactorGraph
     * @param values  Aktuelle Schätzwerte (optimiert oder initial)
     * @param label   Optionaler Freitext (z.B. "keyframe 42")
     */
    bool publish(const gtsam::NonlinearFactorGraph& graph,
                 const gtsam::Values& values,
                 const std::string& label = "") {
        return send(gviz_ipc::MsgType::Replace,
                    buildVars(values), buildEdges(graph, values), label);
    }

    /**
     * Nur Positionen updaten — Topology (Kanten) bleibt unverändert.
     * Ideal nach einem Optimierungsschritt.
     */
    bool publishValuesOnly(const gtsam::Values& values,
                           const std::string& label = "") {
        return send(gviz_ipc::MsgType::ValuesOnly,
                    buildVars(values), {}, label);
    }

    /**
     * Neue Knoten/Kanten inkrementell anhängen (kein Reset).
     * Geeignet für iSAM2-artige inkrementelle Updates.
     */
    bool append(const gtsam::NonlinearFactorGraph& new_factors,
                const gtsam::Values& new_values,
                const std::string& label = "") {
        return send(gviz_ipc::MsgType::Append,
                    buildVars(new_values),
                    buildEdges(new_factors, new_values), label);
    }

    /**
     * Einzelne Kante direkt einfügen (ohne GTSAM-Objekte).
     * Nützlich wenn du die Kante selbst kennst (z.B. Loop-Closure).
     */
    bool appendEdge(gtsam::Key from, gtsam::Key to,
                    FactorType ftype = FactorType::Between, float error = 0.f) {
        gviz_ipc::GVizEdgeEntry e{};
        e.key_from    = from;
        e.key_to      = to;
        e.factor_type = static_cast<uint8_t>(ftype);
        e.error       = error;
        return send(gviz_ipc::MsgType::Append, {}, {e}, "");
    }

    /// Alles in der GUI leeren.
    bool clear() {
        return send(gviz_ipc::MsgType::Clear, {}, {}, "");
    }

private:

    // ── Serialisierung ────────────────────────────────────────────────────────

    static gviz_ipc::VarType detectVarType(gtsam::Key key,
                                           const gtsam::Values& v) {
        try { v.at<gtsam::Pose3>(key);  return gviz_ipc::VarType::Pose3;  } catch(...) {}
        try { v.at<gtsam::Pose2>(key);  return gviz_ipc::VarType::Pose2;  } catch(...) {}
        try { v.at<gtsam::Point3>(key); return gviz_ipc::VarType::Point3; } catch(...) {}
        try { v.at<gtsam::Point2>(key); return gviz_ipc::VarType::Point2; } catch(...) {}
        return gviz_ipc::VarType::Unknown;
    }

    static std::vector<gviz_ipc::GVizVarEntry>
    buildVars(const gtsam::Values& values) {
        std::vector<gviz_ipc::GVizVarEntry> out;
        out.reserve(values.size());

        for (const auto& kv : values) {
            gviz_ipc::GVizVarEntry e{};
            e.key      = kv.key;
            e.var_type = static_cast<uint8_t>(detectVarType(kv.key, values));

            // Extract transform + position
            switch (static_cast<gviz_ipc::VarType>(e.var_type)) {

            case gviz_ipc::VarType::Pose3: {
                auto p = values.at<gtsam::Pose3>(kv.key);
                auto t = p.translation();
                auto R = p.rotation().matrix();
                e.position[0] = (float)t.x();
                e.position[1] = (float)t.y();
                e.position[2] = (float)t.z();
                // Column-major 4×4
                e.transform[0]=(float)R(0,0); e.transform[1]=(float)R(1,0); e.transform[2]=(float)R(2,0);  e.transform[3]=0;
                e.transform[4]=(float)R(0,1); e.transform[5]=(float)R(1,1); e.transform[6]=(float)R(2,1);  e.transform[7]=0;
                e.transform[8]=(float)R(0,2); e.transform[9]=(float)R(1,2); e.transform[10]=(float)R(2,2); e.transform[11]=0;
                e.transform[12]=(float)t.x();e.transform[13]=(float)t.y(); e.transform[14]=(float)t.z();   e.transform[15]=1;
                break;
            }
            case gviz_ipc::VarType::Pose2: {
                auto p = values.at<gtsam::Pose2>(kv.key);
                float c = std::cos((float)p.theta()), s = std::sin((float)p.theta());
                e.position[0] = (float)p.x(); e.position[1] = (float)p.y(); e.position[2] = 0;
                e.transform[0]=c;  e.transform[1]=s;  e.transform[2]=0;  e.transform[3]=0;
                e.transform[4]=-s; e.transform[5]=c;  e.transform[6]=0;  e.transform[7]=0;
                e.transform[8]=0;  e.transform[9]=0;  e.transform[10]=1; e.transform[11]=0;
                e.transform[12]=(float)p.x(); e.transform[13]=(float)p.y(); e.transform[14]=0; e.transform[15]=1;
                break;
            }
            case gviz_ipc::VarType::Point3: {
                auto p = values.at<gtsam::Point3>(kv.key);
                e.position[0]=(float)p.x(); e.position[1]=(float)p.y(); e.position[2]=(float)p.z();
                // Identity rotation, translation only
                e.transform[0]=1; e.transform[5]=1; e.transform[10]=1;
                e.transform[12]=(float)p.x(); e.transform[13]=(float)p.y(); e.transform[14]=(float)p.z(); e.transform[15]=1;
                break;
            }
            case gviz_ipc::VarType::Point2: {
                auto p = values.at<gtsam::Point2>(kv.key);
                e.position[0]=(float)p.x(); e.position[1]=(float)p.y(); e.position[2]=0;
                e.transform[0]=1; e.transform[5]=1; e.transform[10]=1;
                e.transform[12]=(float)p.x(); e.transform[13]=(float)p.y(); e.transform[15]=1;
                break;
            }
            default:
                e.transform[0]=1; e.transform[5]=1; e.transform[10]=1; e.transform[15]=1;
                break;
            }
            out.push_back(e);
        }
        return out;
    }

    static std::vector<gviz_ipc::GVizEdgeEntry>
    buildEdges(const gtsam::NonlinearFactorGraph& graph,
               const gtsam::Values& values) {
        std::vector<gviz_ipc::GVizEdgeEntry> out;

        for (size_t i = 0; i < graph.size(); ++i) {
            auto& f = graph[i];
            if (!f || f->keys().size() < 2) continue;

            auto& keys = f->keys();
            for (size_t k = 0; k+1 < keys.size(); ++k) {
                gviz_ipc::GVizEdgeEntry e{};
                e.key_from = keys[k];
                e.key_to   = keys[k+1];

                // Factor type heuristic
                std::string tn = typeid(*f).name();
                if (tn.find("Prior")   != std::string::npos) e.factor_type = (uint8_t)FactorType::Prior;
                else if (tn.find("Between") != std::string::npos) e.factor_type = (uint8_t)FactorType::Between;
                else if (tn.find("Project") != std::string::npos) e.factor_type = (uint8_t)FactorType::Projection;
                else e.factor_type = (uint8_t)FactorType::Custom;

                // Error (0 if values incomplete)
                try {
                    if (!values.empty()) e.error = (float)f->error(values);
                } catch (...) {}

                out.push_back(e);
            }
        }
        return out;
    }

    // ── Socket I/O ────────────────────────────────────────────────────────────

    bool send(gviz_ipc::MsgType type,
              std::vector<gviz_ipc::GVizVarEntry>  vars,
              std::vector<gviz_ipc::GVizEdgeEntry> edges,
              const std::string& label) {

        std::lock_guard<std::mutex> g(mtx_);

        // Auto-reconnect once if disconnected
        if (fd_ < 0) {
            if (!reconnect()) return false;
        }

        gviz_ipc::GVizMsgHeader hdr{};
        hdr.magic     = gviz_ipc::MAGIC;
        hdr.type      = static_cast<uint8_t>(type);
        hdr.seq_id    = ++seq_;
        hdr.n_vars    = (uint32_t)vars.size();
        hdr.n_edges   = (uint32_t)edges.size();
        hdr.label_len = (uint16_t)std::min(label.size(), (size_t)65535);

        // Build contiguous send buffer
        size_t total = sizeof(hdr)
                     + hdr.label_len
                     + vars.size()  * sizeof(gviz_ipc::GVizVarEntry)
                     + edges.size() * sizeof(gviz_ipc::GVizEdgeEntry);

        buf_.resize(total);
        char* p = buf_.data();
        std::memcpy(p, &hdr, sizeof(hdr));              p += sizeof(hdr);
        std::memcpy(p, label.data(), hdr.label_len);     p += hdr.label_len;
        if (!vars.empty()) {
            size_t sz = vars.size() * sizeof(gviz_ipc::GVizVarEntry);
            std::memcpy(p, vars.data(), sz);             p += sz;
        }
        if (!edges.empty()) {
            size_t sz = edges.size() * sizeof(gviz_ipc::GVizEdgeEntry);
            std::memcpy(p, edges.data(), sz);
        }

        // Send (MSG_NOSIGNAL: don't raise SIGPIPE on broken pipe)
        ssize_t sent = ::send(fd_, buf_.data(), total, MSG_NOSIGNAL);
        if (sent < 0 || (size_t)sent != total) {
            ::close(fd_); fd_ = -1;   // mark disconnected, try next time
            return false;
        }
        return true;
    }

    bool reconnect() {
        // Called while mtx_ is held
        if (!socketPath_.empty()) {
            fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
            if (fd_ < 0) return false;
            struct sockaddr_un addr{};
            addr.sun_family = AF_UNIX;
            std::strncpy(addr.sun_path, socketPath_.c_str(), sizeof(addr.sun_path)-1);
            if (::connect(fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
                ::close(fd_); fd_ = -1; return false;
            }
            return true;
        }
        return false;
    }

    mutable std::mutex  mtx_;
    int                 fd_  = -1;
    uint32_t            seq_ = 0;
    std::string         socketPath_;
    std::vector<char>   buf_;   // reusable send buffer
};
