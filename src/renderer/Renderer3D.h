#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <optional>
#include <vector>
#include "../graph/FactorGraphState.h"
#include "../graph/ResidualColorScale.h"

namespace gtsam_viz {

struct Camera {
    float yaw        = 180.f;
    float pitch      = 30.f;
    float distance   = 15.f;
    glm::vec3 target = {0, 0, 0};
    float fov        = 45.f;

    glm::mat4 view()             const;
    glm::mat4 projection(float aspect) const;
    glm::vec3 position()         const;
    void orbit(float dYaw, float dPitch);
    void zoom(float delta);
    void pan(glm::vec2 delta);
    void moveGround(float forward, float right);
};

class Renderer3D {
public:
    Renderer3D();
    ~Renderer3D();

    bool init();
    void shutdown();
    void resize(int width, int height);
    GLuint render(const FactorGraphState& state);

    Camera& camera() { return cam_; }
    const ResidualStats& residualStats() const { return lastResidualStats_; }

    // ── View toggles ──────────────────────────────────────────────────────────
    bool  showAxes          = true;
    bool  showTrajectory    = true;
    bool  showCovEllipsoids = false;
    bool  showPointClouds   = true;
    bool  showPrimitives    = true;
    bool  followMode        = false;
    bool  colorEdgesByError = true;
    std::optional<size_t> selectedFactor;

    float axisLength        = 0.5f;
    float nodeSize          = 0.12f;
    float edgeErrorScale    = 1.0f;      ///< Larger values make edges turn red sooner
    float covScale          = 3.0f;      ///< σ multiplier for covariance ellipsoids
    float globalPointSize   = 0.f;       ///< 0 = use per-cloud size

    glm::vec4 bgColor = {0.120f, 0.118f, 0.118f, 1.f};

    // ── Grid settings ─────────────────────────────────────────────────────────
    int   gridHalf = 30;
    float gridStep = 1.0f;
    void  rebuildGrid();

private:
    // ── Shaders ───────────────────────────────────────────────────────────────
    void buildShaders();
    GLuint compileShader(GLenum type, const char* src);
    GLuint linkProgram(GLuint vert, GLuint frag);

    // Cached uniform locations (filled once in buildShaders/init)
    struct PhongLoc  { GLint mvp, model, normalMat, color, lightDir; };
    struct FlatLoc   { GLint mvp, color; };
    struct LineLoc   { GLint vp; };
    struct PointLoc  { GLint vp, pointSize, defaultColor; };

    PhongLoc  phongLoc_{};
    FlatLoc   flatLoc_{};
    LineLoc   lineLoc_{};
    PointLoc  pointLoc_{};

    // ── Geometry builders ─────────────────────────────────────────────────────
    void buildGridMesh();
    void buildAxisMesh();
    void buildSphereMesh();

    // ── Per-frame draw helpers ─────────────────────────────────────────────────
    void drawGrid(const glm::mat4& VP);
    void drawPoseAxes(const glm::mat4& transform, float scale, const glm::mat4& VP);
    void drawSphere(glm::vec3 pos, float rx, float ry, float rz,
                    const glm::mat4& rotation, glm::vec4 color, const glm::mat4& VP);
    glm::vec4 edgeColor(const FactorNode& factor, double scale) const;

    // Solid primitive geometry generation (appends triangles to solidBuffer_)
    void appendCone(glm::vec3 apex, glm::vec3 base_center, float radius);
    void appendCylinder(glm::vec3 center, glm::vec3 axis, float radius, float half_height);
    void appendBox(glm::vec3 center, glm::vec3 half_extents, const glm::mat3& rot);
    void appendArrowHead(glm::vec3 tip, glm::vec3 shaft_dir, float head_len, float head_radius);

    // Flush solid buffer to GPU and draw with phong shader
    void flushSolid(glm::vec4 color, const glm::mat4& VP);

    // ── FBO ───────────────────────────────────────────────────────────────────
    GLuint fbo_ = 0, colorTex_ = 0, depthRbo_ = 0;
    int    fboW_ = 0, fboH_ = 0;

    // ── Shader programs ───────────────────────────────────────────────────────
    GLuint phongProg_ = 0, flatProg_ = 0, lineProg_ = 0, pointProg_ = 0;

    // ── Static meshes ─────────────────────────────────────────────────────────
    GLuint gridVao_ = 0,   gridVbo_ = 0;   int gridVerts_ = 0;
    GLuint axisVao_ = 0,   axisVbo_ = 0;
    GLuint sphereVao_ = 0, sphereVbo_ = 0, sphereEbo_ = 0;
    int    sphereIdxCount_ = 0;

    // ── Dynamic line buffer ───────────────────────────────────────────────────
    struct LineVertex { glm::vec3 pos; glm::vec4 color; };
    std::vector<LineVertex> lineBuffer_;
    std::vector<LineVertex> selectedLineBuffer_;
    GLuint lineVao_ = 0, lineVbo_ = 0;

    // ── Dynamic solid buffer (pos+norm, rebuilt per frame) ────────────────────
    struct SolidVertex { glm::vec3 pos; glm::vec3 norm; };
    std::vector<SolidVertex> solidBuffer_;
    GLuint solidVao_ = 0, solidVbo_ = 0;

    // ── Point cloud VAO/VBO (rebuilt when cloud data changes) ─────────────────
    GLuint pcVao_ = 0, pcVbo_ = 0, pcColorVbo_ = 0;

    Camera cam_;
    bool   initialized_  = false;
    int    lastGridHalf_ = -1;
    float  lastGridStep_ = -1.f;
    ResidualStats lastResidualStats_;

    // ── Uniform helpers ───────────────────────────────────────────────────────
    void setMat4(GLint loc, const glm::mat4& m);
    void setVec4(GLint loc, const glm::vec4& v);
    void setVec3(GLint loc, const glm::vec3& v);
    void setFloat(GLint loc, float v);
};

} // namespace gtsam_viz
