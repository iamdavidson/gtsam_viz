#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include "../graph/FactorGraphState.h"

namespace gtsam_viz {

struct Camera {
    float yaw        = -90.f;
    float pitch      = 30.f;
    float distance   = 15.f;
    glm::vec3 target = {0,0,0};
    float fov        = 45.f;

    glm::mat4 view()       const;
    glm::mat4 projection(float aspect) const;
    glm::vec3 position()   const;
    void orbit(float dYaw, float dPitch);
    void zoom(float delta);
    void pan(glm::vec2 delta);
    /// Move target in the horizontal ground plane (WASD / arrow keys)
    void moveGround(float forward, float right);
};

class Renderer3D {
public:
    Renderer3D();
    ~Renderer3D();

    bool init();
    void shutdown();
    void resize(int width, int height);
    /// Render scene; returns texture ID for ImGui::Image
    GLuint render(const FactorGraphState& state);

    Camera& camera() { return cam_; }

    // ── View toggles ──────────────────────────────────────────────────────────
    bool  showAxes          = true;
    bool  showTrajectory    = true;
    bool  showCovEllipsoids = false;
    bool  followMode        = false;   ///< Camera target tracks latest pose
    float axisLength        = 0.5f;
    float nodeSize          = 0.12f;
    float covScale          = 3.0f;    ///< σ multiplier for covariance ellipsoids
    glm::vec4 bgColor       = {0.07f, 0.08f, 0.10f, 1.f};

    // ── Grid settings ─────────────────────────────────────────────────────────
    int   gridHalf          = 30;      ///< Half-extent in grid steps
    float gridStep          = 1.0f;    ///< Step size in world units

    /// Call when gridHalf or gridStep change to rebuild the GPU mesh
    void rebuildGrid();

private:
    void buildShaders();
    void buildGridMesh();
    void buildAxisMesh();
    void buildSphereMesh();

    void drawGrid();
    void drawPoseAxes(const glm::mat4& transform, float scale);
    void drawEdge(glm::vec3 a, glm::vec3 b, glm::vec4 color);
    void drawSphere(glm::vec3 pos, float rx, float ry, float rz,
                    const glm::mat4& rotation, glm::vec4 color);

    // FBO
    GLuint fbo_      = 0, colorTex_ = 0, depthRbo_ = 0;
    int    fboW_     = 0, fboH_     = 0;

    // Shaders
    GLuint phongProg_ = 0, flatProg_ = 0, lineProg_ = 0;

    // Geometry
    GLuint gridVao_ = 0, gridVbo_ = 0;  int gridVerts_ = 0;
    GLuint axisVao_ = 0, axisVbo_ = 0;
    GLuint sphereVao_ = 0, sphereVbo_ = 0, sphereEbo_ = 0;
    int    sphereIdxCount_ = 0;

    Camera cam_;

    struct LineVertex { glm::vec3 pos; glm::vec4 color; };
    std::vector<LineVertex> lineBuffer_;
    GLuint lineVao_ = 0, lineVbo_ = 0;

    bool initialized_ = false;

    // Cached grid params to detect changes
    int   lastGridHalf_ = -1;
    float lastGridStep_ = -1.f;

    GLuint compileShader(GLenum type, const char* src);
    GLuint linkProgram(GLuint vert, GLuint frag);
    void   setUniformMat4(GLuint prog, const char* name, const glm::mat4& m);
    void   setUniformVec4(GLuint prog, const char* name, const glm::vec4& v);
    void   setUniformVec3(GLuint prog, const char* name, const glm::vec3& v);
    void   setUniformFloat(GLuint prog, const char* name, float v);
};

} // namespace gtsam_viz
