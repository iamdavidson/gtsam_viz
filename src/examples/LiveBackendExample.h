#pragma once
#include <thread>

namespace gtsam_viz {

/**
 * Fertige Demo-Backends für die Live-Bridge.
 * Starten einen eigenen Thread, der den GraphBridge-Singleton befüllt.
 *
 * Verwendung in main.cpp oder nach Button-Klick in der GUI:
 *
 *   auto t = gtsam_viz::LiveBackendExample::startGrowingChain(4.0);
 *   t.detach();  // oder am Ende t.join()
 */
struct LiveBackendExample {
    /** Wachsende Pose2-Kette mit Odometrie + Loop-Closure alle 25 Schritte. */
    static std::thread startGrowingChain(double hz = 4.0);

    /** Inkrementeller iSAM2-Loop mit Pose2-Trajektorie. */
    static std::thread startISAM2(double hz = 5.0);
};

} // namespace gtsam_viz
