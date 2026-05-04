#include "Renderer3D.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <cstring>
#include <algorithm>
#include <unordered_map>

namespace gtsam_viz {

// ── GLSL shaders ──────────────────────────────────────────────────────────────

static const char* PHONG_VERT = R"(
#version 430 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNorm;
uniform mat4 uMVP;
uniform mat4 uModel;
uniform mat3 uNormalMat;
out vec3 vNormal; out vec3 vFragPos;
void main(){
    vFragPos  = vec3(uModel * vec4(aPos,1.0));
    vNormal   = normalize(uNormalMat * aNorm);
    gl_Position = uMVP * vec4(aPos,1.0);
})";

static const char* PHONG_FRAG = R"(
#version 430 core
in vec3 vNormal; in vec3 vFragPos;
uniform vec4 uColor; uniform vec3 uLightDir;
out vec4 FragColor;
void main(){
    float ambient = 0.25;
    float diffuse = max(dot(normalize(vNormal), normalize(uLightDir)), 0.0);
    FragColor = vec4(uColor.rgb * (ambient + 0.75*diffuse), uColor.a);
})";

static const char* FLAT_VERT = R"(
#version 430 core
layout(location=0) in vec3 aPos;
uniform mat4 uMVP;
void main(){ gl_Position = uMVP * vec4(aPos,1.0); })";

static const char* FLAT_FRAG = R"(
#version 430 core
uniform vec4 uColor; out vec4 FragColor;
void main(){ FragColor = uColor; })";

static const char* LINE_VERT = R"(
#version 430 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec4 aColor;
uniform mat4 uVP; out vec4 vColor;
void main(){ vColor=aColor; gl_Position = uVP * vec4(aPos,1.0); })";

static const char* LINE_FRAG = R"(
#version 430 core
in vec4 vColor; out vec4 FragColor;
void main(){ FragColor = vColor; })";

// Point cloud shader — uses gl_PointSize for round sprite points
static const char* POINT_VERT = R"(
#version 430 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec4 aColor;
uniform mat4  uVP;
uniform float uPointSize;
uniform vec4  uDefaultColor;
out vec4 vColor;
void main(){
    vColor = (aColor.a > 0.0) ? aColor : uDefaultColor;
    gl_PointSize = uPointSize;
    gl_Position  = uVP * vec4(aPos, 1.0);
})";

static const char* POINT_FRAG = R"(
#version 430 core
in vec4 vColor; out vec4 FragColor;
void main(){
    // Circular points
    vec2 c = gl_PointCoord * 2.0 - 1.0;
    if (dot(c,c) > 1.0) discard;
    FragColor = vColor;
})";

static Eigen::Matrix3f covarianceToRenderCoords(const Eigen::Matrix3f& cov) {
    Eigen::Matrix3f swap;
    swap << 1.f, 0.f, 0.f,
            0.f, 0.f, 1.f,
            0.f, 1.f, 0.f;
    return swap * cov * swap.transpose();
}

// ── Camera ────────────────────────────────────────────────────────────────────

glm::mat4 Camera::view() const {
    return glm::lookAt(position(), target, glm::vec3(0,1,0));
}
glm::mat4 Camera::projection(float aspect) const {
    return glm::perspective(glm::radians(fov), aspect, 0.01f, 2000.f);
}
glm::vec3 Camera::position() const {
    float yR = glm::radians(yaw), pR = glm::radians(pitch);
    return target + distance * glm::vec3(
        std::cos(pR)*std::cos(yR),
        std::sin(pR),
        std::cos(pR)*std::sin(yR));
}
void Camera::orbit(float dYaw, float dPitch) {
    yaw  += dYaw;
    pitch = glm::clamp(pitch + dPitch, -89.f, 89.f);
}
void Camera::zoom(float delta) {
    distance = glm::clamp(distance - delta, 0.1f, 2000.f);
}
void Camera::pan(glm::vec2 delta) {
    glm::vec3 front = glm::normalize(target - position());
    glm::vec3 right = glm::normalize(glm::cross(front, glm::vec3(0,1,0)));
    glm::vec3 up    = glm::normalize(glm::cross(right, front));
    target += right * (-delta.x * 0.01f * distance)
            + up    * ( delta.y * 0.01f * distance);
}
void Camera::moveGround(float forward, float right_) {
    float yR = glm::radians(yaw);
    glm::vec3 fwd  = glm::normalize(glm::vec3(std::cos(yR), 0, std::sin(yR)));
    glm::vec3 rght = glm::normalize(glm::cross(fwd, glm::vec3(0,1,0)));
    float speed    = distance * 0.8f;
    target += fwd * forward * speed + rght * right_ * speed;
}

// ── Renderer3D ────────────────────────────────────────────────────────────────

Renderer3D::Renderer3D()  = default;
Renderer3D::~Renderer3D() { shutdown(); }

// ── Uniform helpers (use cached locations — no string lookup per frame) ────────
void Renderer3D::setMat4(GLint loc, const glm::mat4& m) {
    glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(m));
}
void Renderer3D::setVec4(GLint loc, const glm::vec4& v) {
    glUniform4fv(loc, 1, glm::value_ptr(v));
}
void Renderer3D::setVec3(GLint loc, const glm::vec3& v) {
    glUniform3fv(loc, 1, glm::value_ptr(v));
}
void Renderer3D::setFloat(GLint loc, float v) {
    glUniform1f(loc, v);
}

GLuint Renderer3D::compileShader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char buf[512]; glGetShaderInfoLog(s, 512, nullptr, buf);
        glDeleteShader(s);
        throw std::runtime_error(std::string("Shader compile: ") + buf);
    }
    return s;
}
GLuint Renderer3D::linkProgram(GLuint v, GLuint f) {
    GLuint p = glCreateProgram();
    glAttachShader(p, v); glAttachShader(p, f);
    glLinkProgram(p);
    GLint ok; glGetProgramiv(p, GL_LINK_STATUS, &ok);
    glDeleteShader(v); glDeleteShader(f);
    if (!ok) {
        char buf[512]; glGetProgramInfoLog(p, 512, nullptr, buf);
        glDeleteProgram(p);
        throw std::runtime_error(std::string("Program link: ") + buf);
    }
    return p;
}

bool Renderer3D::init() {
    if (initialized_) return true;
    try {
        buildShaders();
        buildGridMesh();
        buildAxisMesh();
        buildSphereMesh();

        // Line VAO
        glGenVertexArrays(1, &lineVao_); glGenBuffers(1, &lineVbo_);
        glBindVertexArray(lineVao_);
        glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(LineVertex),(void*)offsetof(LineVertex,pos));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1,4,GL_FLOAT,GL_FALSE,sizeof(LineVertex),(void*)offsetof(LineVertex,color));
        glBindVertexArray(0);

        // Solid VAO (pos+norm, no EBO — all triangle strips built in CPU)
        glGenVertexArrays(1, &solidVao_); glGenBuffers(1, &solidVbo_);
        glBindVertexArray(solidVao_);
        glBindBuffer(GL_ARRAY_BUFFER, solidVbo_);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(SolidVertex),(void*)offsetof(SolidVertex,pos));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(SolidVertex),(void*)offsetof(SolidVertex,norm));
        glBindVertexArray(0);

        // Point cloud VAO — pos (binding 0) + color (binding 1, optional)
        glGenVertexArrays(1, &pcVao_);
        glGenBuffers(1, &pcVbo_);
        glGenBuffers(1, &pcColorVbo_);
        glBindVertexArray(pcVao_);
        glBindBuffer(GL_ARRAY_BUFFER, pcVbo_);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,12,nullptr);
        // Color VBO: RGBA8 normalized
        glBindBuffer(GL_ARRAY_BUFFER, pcColorVbo_);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1,4,GL_UNSIGNED_BYTE,GL_TRUE,4,nullptr);
        glBindVertexArray(0);

        glEnable(GL_PROGRAM_POINT_SIZE);

        initialized_ = true;
        return true;
    } catch (...) { return false; }
}

void Renderer3D::shutdown() {
    if (!initialized_) return;
    if (fbo_)      glDeleteFramebuffers(1,  &fbo_);
    if (colorTex_) glDeleteTextures(1,      &colorTex_);
    if (depthRbo_) glDeleteRenderbuffers(1, &depthRbo_);
    glDeleteVertexArrays(1,&gridVao_);    glDeleteBuffers(1,&gridVbo_);
    glDeleteVertexArrays(1,&axisVao_);    glDeleteBuffers(1,&axisVbo_);
    glDeleteVertexArrays(1,&sphereVao_);  glDeleteBuffers(1,&sphereVbo_);
    glDeleteBuffers(1,&sphereEbo_);
    glDeleteVertexArrays(1,&lineVao_);    glDeleteBuffers(1,&lineVbo_);
    glDeleteVertexArrays(1,&solidVao_);   glDeleteBuffers(1,&solidVbo_);
    glDeleteVertexArrays(1,&pcVao_);      glDeleteBuffers(1,&pcVbo_);
    glDeleteBuffers(1,&pcColorVbo_);
    glDeleteProgram(phongProg_);
    glDeleteProgram(flatProg_);
    glDeleteProgram(lineProg_);
    glDeleteProgram(pointProg_);
    initialized_ = false;
}

void Renderer3D::resize(int w, int h) {
    if (w == fboW_ && h == fboH_) return;
    fboW_ = w; fboH_ = h;
    if (fbo_)      glDeleteFramebuffers(1,  &fbo_);
    if (colorTex_) glDeleteTextures(1,      &colorTex_);
    if (depthRbo_) glDeleteRenderbuffers(1, &depthRbo_);

    glGenFramebuffers(1, &fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    glGenTextures(1, &colorTex_);
    glBindTexture(GL_TEXTURE_2D, colorTex_);
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,w,h,0,GL_RGBA,GL_UNSIGNED_BYTE,nullptr);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,GL_TEXTURE_2D,colorTex_,0);

    glGenRenderbuffers(1,&depthRbo_);
    glBindRenderbuffer(GL_RENDERBUFFER,depthRbo_);
    glRenderbufferStorage(GL_RENDERBUFFER,GL_DEPTH24_STENCIL8,w,h);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER,GL_DEPTH_STENCIL_ATTACHMENT,GL_RENDERBUFFER,depthRbo_);
    glBindFramebuffer(GL_FRAMEBUFFER,0);
}

// ── Main render ───────────────────────────────────────────────────────────────

GLuint Renderer3D::render(const FactorGraphState& state) {
    if (!initialized_ || fboW_==0 || fboH_==0) return 0;

    if (gridHalf != lastGridHalf_ || gridStep != lastGridStep_)
        rebuildGrid();

    if (followMode && !state.variables().empty())
        cam_.target = state.variables().back().position3d;

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glViewport(0, 0, fboW_, fboH_);
    glClearColor(bgColor.r, bgColor.g, bgColor.b, bgColor.a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    float     aspect = float(fboW_) / float(fboH_);
    glm::mat4 VP     = cam_.projection(aspect) * cam_.view();

    auto& vars    = state.variables();
    auto& factors = state.factors();

    // O(1) key→var lookup for trajectory drawing
    std::unordered_map<gtsam::Key, const VariableNode*> keyToVar;
    keyToVar.reserve(vars.size());
    for (auto& vn : vars) keyToVar[vn.key] = &vn;

    drawGrid(VP);

    lineBuffer_.clear();

    // ── Trajectory edges ──────────────────────────────────────────────────────
    if (showTrajectory) {
        for (auto& fn : factors) {
            if (fn.keys.size() < 2) continue;
            for (size_t i = 0; i+1 < fn.keys.size(); ++i) {
                auto itA = keyToVar.find(fn.keys[i]);
                auto itB = keyToVar.find(fn.keys[i+1]);
                if (itA == keyToVar.end() || itB == keyToVar.end()) continue;
                glm::vec4 col = colorEdgesByError
                    ? edgeColor(fn.error)
                    : glm::vec4{0.3f, 0.9f, 0.5f, 0.9f};
                lineBuffer_.push_back({itA->second->position3d, col});
                lineBuffer_.push_back({itB->second->position3d, col});
            }
        }
    }

    // ── Graph nodes ───────────────────────────────────────────────────────────
    for (auto& vn : vars) {
        bool isPose = vn.type==VariableType::Pose2 || vn.type==VariableType::Pose3;
        if (showAxes) drawPoseAxes(vn.transform, axisLength, VP);
        drawSphere(vn.position3d, nodeSize, nodeSize, nodeSize,
                   glm::mat4(1.f),
                   isPose ? glm::vec4{0.3f,0.65f,1.f,1.f}
                          : glm::vec4{0.3f,1.f,0.6f,1.f},
                   VP);
    }

    // ── Covariance ellipsoids ─────────────────────────────────────────────────
    if (showCovEllipsoids) {
        for (auto& vn : vars) {
            Eigen::Matrix3f C = Eigen::Matrix3f::Zero();
            bool haveCov = false;

            // Priority 1: IPC-provided covariance on the variable node
            if (vn.has_covariance) {
                // Row-major 3×3 stored in covariance3d
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        C(r,c) = vn.covariance3d[r*3+c];
                haveCov = true;
            }
            // Priority 2: GTSAM marginals from in-viewer optimizer
            if (!haveCov) {
                auto covOpt = state.marginalCovariance(vn.key);
                if (covOpt) {
                    const gtsam::Matrix& M = *covOpt;
                    int off = (M.rows() == 6) ? 3 : 0;
                    C = M.block<3,3>(off,off).cast<float>();
                    C = covarianceToRenderCoords(C);
                    haveCov = true;
                }
            }
            if (!haveCov) continue;

            // Eigendecomposition of symmetric 3×3 covariance
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(C);
            if (solver.info() != Eigen::Success) continue;

            Eigen::Vector3f ev = solver.eigenvalues();
            Eigen::Matrix3f V  = solver.eigenvectors(); // columns = axes

            // Guard against negative/tiny eigenvalues
            float rx = std::sqrt(std::max(ev[0], 1e-8f)) * covScale;
            float ry = std::sqrt(std::max(ev[1], 1e-8f)) * covScale;
            float rz = std::sqrt(std::max(ev[2], 1e-8f)) * covScale;

            // Build rotation from eigenvectors (columns of V)
            glm::mat4 rot = glm::mat4(
                V(0,0), V(1,0), V(2,0), 0,
                V(0,1), V(1,1), V(2,1), 0,
                V(0,2), V(1,2), V(2,2), 0,
                0,      0,      0,      1);

            drawSphere(vn.position3d, rx, ry, rz, rot,
                       {0.95f, 0.75f, 0.15f, 0.22f}, VP);
        }
    }

    // ── Primitives ────────────────────────────────────────────────────────────
    if (showPrimitives) {
        for (auto& prim : state.primitives()) {
            const float* d = prim.data;
            glm::vec4    col = prim.color;

            switch (prim.type) {

            case PrimType::Line: {
                glm::vec3 from{d[0],d[1],d[2]}, to{d[3],d[4],d[5]};
                lineBuffer_.push_back({from, col});
                lineBuffer_.push_back({to,   col});
                break;
            }

            case PrimType::Arrow: {
                glm::vec3 from{d[0],d[1],d[2]}, to{d[3],d[4],d[5]};
                glm::vec3 dir = to - from;
                float len = glm::length(dir);
                if (len < 1e-6f) break;
                glm::vec3 unit = dir / len;
                float headLen    = len * 0.2f;
                float headRadius = headLen * 0.35f;
                glm::vec3 shaftEnd = to - unit * headLen;
                lineBuffer_.push_back({from,     col});
                lineBuffer_.push_back({shaftEnd, col});
                solidBuffer_.clear();
                appendArrowHead(to, unit, headLen, headRadius);
                flushSolid(col, VP);
                break;
            }

            case PrimType::Box: {
                glm::vec3 center{d[0],d[1],d[2]}, he{d[3],d[4],d[5]};
                glm::mat3 rot(d[6],d[7],d[8], d[9],d[10],d[11], d[12],d[13],d[14]);
                solidBuffer_.clear();
                appendBox(center, he, rot);
                flushSolid(col, VP);
                break;
            }

            case PrimType::Sphere: {
                glm::vec3 ctr{d[0],d[1],d[2]};
                float r = d[3];
                drawSphere(ctr, r, r, r, glm::mat4(1.f), col, VP);
                break;
            }

            case PrimType::Cone: {
                glm::vec3 apex{d[0],d[1],d[2]}, base{d[3],d[4],d[5]};
                float br = d[6];
                solidBuffer_.clear();
                appendCone(apex, base, br);
                flushSolid(col, VP);
                break;
            }

            case PrimType::Cylinder: {
                glm::vec3 ctr{d[0],d[1],d[2]}, axis{d[3],d[4],d[5]};
                float r = d[6], hh = d[7];
                solidBuffer_.clear();
                appendCylinder(ctr, axis, r, hh);
                flushSolid(col, VP);
                break;
            }

            case PrimType::CoordFrame: {
                glm::vec3 pos{d[0],d[1],d[2]};
                float axLen = d[3];
                // row-major 3×3 rotation in d[4..12]
                glm::mat4 T(1.f);
                // col 0 = X axis
                T[0] = glm::vec4(d[4], d[7], d[10], 0);
                T[1] = glm::vec4(d[5], d[8], d[11], 0);
                T[2] = glm::vec4(d[6], d[9], d[12], 0);
                T[3] = glm::vec4(pos, 1.f);
                drawPoseAxes(T, axLen, VP);
                break;
            }
            }
        }
    }

    // ── Flush line buffer ─────────────────────────────────────────────────────
    if (!lineBuffer_.empty()) {
        glUseProgram(lineProg_);
        setMat4(lineLoc_.vp, VP);
        glBindVertexArray(lineVao_);
        glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
        glBufferData(GL_ARRAY_BUFFER,
                     lineBuffer_.size()*sizeof(LineVertex),
                     lineBuffer_.data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                              sizeof(LineVertex), (void*)offsetof(LineVertex, pos));
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE,
                              sizeof(LineVertex), (void*)offsetof(LineVertex, color));
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glLineWidth(2.f);
        glDrawArrays(GL_LINES, 0, (GLsizei)lineBuffer_.size());
        glDisable(GL_BLEND);
        glBindVertexArray(0);
    }

    // ── Point clouds ──────────────────────────────────────────────────────────
    if (showPointClouds) {
        glUseProgram(pointProg_);
        setMat4(pointLoc_.vp, VP);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        for (auto& pc : state.pointClouds()) {
            if (pc.points.empty()) continue;

            float ps = (globalPointSize > 0.f) ? globalPointSize : pc.point_size;
            setFloat(pointLoc_.pointSize, ps);
            setVec4(pointLoc_.defaultColor, pc.default_color);

            glBindVertexArray(pcVao_);

            // Upload positions
            glBindBuffer(GL_ARRAY_BUFFER, pcVbo_);
            glBufferData(GL_ARRAY_BUFFER,
                         pc.points.size() * sizeof(glm::vec3),
                         pc.points.data(), GL_DYNAMIC_DRAW);

            // Upload colors (or zero-alpha dummy so shader uses defaultColor)
            glBindBuffer(GL_ARRAY_BUFFER, pcColorVbo_);
            if (!pc.colors.empty()) {
                glBufferData(GL_ARRAY_BUFFER,
                             pc.colors.size() * sizeof(glm::u8vec4),
                             pc.colors.data(), GL_DYNAMIC_DRAW);
            } else {
                // One transparent pixel → shader branches to defaultColor
                static const glm::u8vec4 DUMMY{0,0,0,0};
                glBufferData(GL_ARRAY_BUFFER, sizeof(DUMMY), &DUMMY, GL_DYNAMIC_DRAW);
                // Bind with stride=0 so all vertices read the same value
                glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, 0, nullptr);
            }

            glDrawArrays(GL_POINTS, 0, (GLsizei)pc.points.size());
            glBindVertexArray(0);

            // Restore color VBO binding for next cloud
            glBindVertexArray(pcVao_);
            glBindBuffer(GL_ARRAY_BUFFER, pcColorVbo_);
            glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, 4, nullptr);
            glBindVertexArray(0);
        }
        glDisable(GL_BLEND);
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return colorTex_;
}

// ── Mesh builders ─────────────────────────────────────────────────────────────

void Renderer3D::buildShaders() {
    phongProg_  = linkProgram(compileShader(GL_VERTEX_SHADER, PHONG_VERT),
                              compileShader(GL_FRAGMENT_SHADER, PHONG_FRAG));
    flatProg_   = linkProgram(compileShader(GL_VERTEX_SHADER, FLAT_VERT),
                              compileShader(GL_FRAGMENT_SHADER, FLAT_FRAG));
    lineProg_   = linkProgram(compileShader(GL_VERTEX_SHADER, LINE_VERT),
                              compileShader(GL_FRAGMENT_SHADER, LINE_FRAG));
    pointProg_  = linkProgram(compileShader(GL_VERTEX_SHADER, POINT_VERT),
                              compileShader(GL_FRAGMENT_SHADER, POINT_FRAG));

    // Cache uniform locations (one-time string lookup per program)
    phongLoc_.mvp       = glGetUniformLocation(phongProg_, "uMVP");
    phongLoc_.model     = glGetUniformLocation(phongProg_, "uModel");
    phongLoc_.normalMat = glGetUniformLocation(phongProg_, "uNormalMat");
    phongLoc_.color     = glGetUniformLocation(phongProg_, "uColor");
    phongLoc_.lightDir  = glGetUniformLocation(phongProg_, "uLightDir");

    flatLoc_.mvp   = glGetUniformLocation(flatProg_, "uMVP");
    flatLoc_.color = glGetUniformLocation(flatProg_, "uColor");

    lineLoc_.vp    = glGetUniformLocation(lineProg_, "uVP");

    pointLoc_.vp           = glGetUniformLocation(pointProg_, "uVP");
    pointLoc_.pointSize    = glGetUniformLocation(pointProg_, "uPointSize");
    pointLoc_.defaultColor = glGetUniformLocation(pointProg_, "uDefaultColor");
}

void Renderer3D::buildGridMesh() {
    lastGridHalf_ = gridHalf;
    lastGridStep_ = gridStep;

    std::vector<float> verts;
    verts.reserve((gridHalf*2+1)*4*3);
    for (int i = -gridHalf; i <= gridHalf; ++i) {
        float fi = float(i) * gridStep, fh = float(gridHalf) * gridStep;
        verts.insert(verts.end(), { fi, 0.f,-fh, fi, 0.f, fh });
        verts.insert(verts.end(), {-fh, 0.f, fi, fh, 0.f, fi });
    }
    gridVerts_ = (int)(verts.size()/3);

    if (!gridVao_) { glGenVertexArrays(1,&gridVao_); glGenBuffers(1,&gridVbo_); }
    glBindVertexArray(gridVao_);
    glBindBuffer(GL_ARRAY_BUFFER, gridVbo_);
    glBufferData(GL_ARRAY_BUFFER, verts.size()*4, verts.data(), GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,12,nullptr);
    glBindVertexArray(0);
}

void Renderer3D::rebuildGrid() {
    if (initialized_) buildGridMesh();
}

void Renderer3D::buildAxisMesh() {
    float verts[] = { 0,0,0, 1,0,0,  0,0,0, 0,1,0,  0,0,0, 0,0,1 };
    glGenVertexArrays(1,&axisVao_); glGenBuffers(1,&axisVbo_);
    glBindVertexArray(axisVao_);
    glBindBuffer(GL_ARRAY_BUFFER,axisVbo_);
    glBufferData(GL_ARRAY_BUFFER,sizeof(verts),verts,GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,12,nullptr);
    glBindVertexArray(0);
}

void Renderer3D::buildSphereMesh() {
    std::vector<float>    verts;
    std::vector<unsigned> idx;
    int stacks=14, slices=18;
    for (int i=0;i<=stacks;++i) {
        float phi = glm::pi<float>() * float(i)/float(stacks);
        for (int j=0;j<=slices;++j) {
            float theta = 2.f*glm::pi<float>() * float(j)/float(slices);
            float x = std::sin(phi)*std::cos(theta);
            float y = std::cos(phi);
            float z = std::sin(phi)*std::sin(theta);
            verts.insert(verts.end(),{x,y,z, x,y,z});
        }
    }
    for (int i=0;i<stacks;++i)
        for (int j=0;j<slices;++j) {
            int a=i*(slices+1)+j, b=a+slices+1;
            idx.insert(idx.end(),{(unsigned)a,(unsigned)b,(unsigned)(a+1),
                                  (unsigned)(a+1),(unsigned)b,(unsigned)(b+1)});
        }
    sphereIdxCount_ = (int)idx.size();
    glGenVertexArrays(1,&sphereVao_); glGenBuffers(1,&sphereVbo_); glGenBuffers(1,&sphereEbo_);
    glBindVertexArray(sphereVao_);
    glBindBuffer(GL_ARRAY_BUFFER,sphereVbo_);
    glBufferData(GL_ARRAY_BUFFER,verts.size()*4,verts.data(),GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,sphereEbo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,idx.size()*4,idx.data(),GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,24,nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,24,(void*)12);
    glBindVertexArray(0);
}

// ── Draw helpers ──────────────────────────────────────────────────────────────

void Renderer3D::drawGrid(const glm::mat4& VP) {
    glUseProgram(flatProg_);
    setMat4(flatLoc_.mvp,   VP);
    setVec4(flatLoc_.color, {0.260f,0.250f,0.250f,1.f});
    glBindVertexArray(gridVao_);
    glLineWidth(1.f);
    glDrawArrays(GL_LINES, 0, gridVerts_);
    glBindVertexArray(0);
}

void Renderer3D::drawPoseAxes(const glm::mat4& transform, float scale, const glm::mat4& VP) {
    glm::mat4 M = transform * glm::scale(glm::mat4(1.f), glm::vec3(scale));
    glLineWidth(3.f);
    glBindVertexArray(axisVao_);
    glUseProgram(flatProg_);
    setMat4(flatLoc_.mvp,   VP * M);
    setVec4(flatLoc_.color, {1.f,0.22f,0.22f,1.f}); glDrawArrays(GL_LINES,0,2);
    setVec4(flatLoc_.color, {0.22f,1.f,0.22f,1.f}); glDrawArrays(GL_LINES,2,2);
    setVec4(flatLoc_.color, {0.22f,0.4f,1.f,1.f});  glDrawArrays(GL_LINES,4,2);
    glBindVertexArray(0);
}

glm::vec4 Renderer3D::edgeColor(double error) const {
    float e = std::max(0.f, static_cast<float>(error));
    float t = 1.f - std::exp(-e * std::max(0.001f, edgeErrorScale));
    t = glm::clamp(t, 0.f, 1.f);

    constexpr glm::vec4 green{0.18f, 0.95f, 0.35f, 0.95f};
    constexpr glm::vec4 yellow{1.00f, 0.88f, 0.15f, 0.95f};
    constexpr glm::vec4 red{1.00f, 0.18f, 0.12f, 0.95f};

    if (t < 0.5f)
        return glm::mix(green, yellow, t * 2.f);
    return glm::mix(yellow, red, (t - 0.5f) * 2.f);
}

void Renderer3D::drawSphere(glm::vec3 pos, float rx, float ry, float rz,
                             const glm::mat4& rotation, glm::vec4 color,
                             const glm::mat4& VP) {
    glm::mat4 T = glm::translate(glm::mat4(1.f), pos);
    glm::mat4 S = glm::scale(glm::mat4(1.f), {rx, ry, rz});
    glm::mat4 M = T * rotation * S;
    glm::mat3 NM = glm::mat3(glm::transpose(glm::inverse(M)));

    bool needBlend = (color.a < 0.99f);
    if (needBlend) { glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); }

    glUseProgram(phongProg_);
    setMat4(phongLoc_.mvp,   VP * M);
    setMat4(phongLoc_.model, M);
    glUniformMatrix3fv(phongLoc_.normalMat, 1, GL_FALSE, glm::value_ptr(NM));
    setVec4(phongLoc_.color,    color);
    setVec3(phongLoc_.lightDir, glm::normalize(glm::vec3{1,3,2}));
    glBindVertexArray(sphereVao_);
    glDrawElements(GL_TRIANGLES, sphereIdxCount_, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);

    if (needBlend) glDisable(GL_BLEND);
}

void Renderer3D::flushSolid(glm::vec4 color, const glm::mat4& VP) {
    if (solidBuffer_.empty()) return;

    glm::mat4 M = glm::mat4(1.f); // identity — geometry is already in world space
    glm::mat3 NM = glm::mat3(1.f);

    bool needBlend = (color.a < 0.99f);
    if (needBlend) { glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); }

    glUseProgram(phongProg_);
    setMat4(phongLoc_.mvp,   VP);
    setMat4(phongLoc_.model, M);
    glUniformMatrix3fv(phongLoc_.normalMat, 1, GL_FALSE, glm::value_ptr(NM));
    setVec4(phongLoc_.color,    color);
    setVec3(phongLoc_.lightDir, glm::normalize(glm::vec3{1,3,2}));

    glBindVertexArray(solidVao_);
    glBindBuffer(GL_ARRAY_BUFFER, solidVbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 solidBuffer_.size() * sizeof(SolidVertex),
                 solidBuffer_.data(), GL_DYNAMIC_DRAW);
    glDrawArrays(GL_TRIANGLES, 0, (GLsizei)solidBuffer_.size());
    glBindVertexArray(0);

    if (needBlend) glDisable(GL_BLEND);
}

// ── Solid geometry generators (append triangles with normals to solidBuffer_) ──

void Renderer3D::appendCone(glm::vec3 apex, glm::vec3 base_center, float radius) {
    constexpr int SEGS = 18;
    glm::vec3 axis = base_center - apex;
    float len = glm::length(axis);
    if (len < 1e-7f) return;
    glm::vec3 Z = axis / len;

    // Build orthonormal frame
    glm::vec3 ref = (std::abs(Z.y) < 0.9f) ? glm::vec3(0,1,0) : glm::vec3(1,0,0);
    glm::vec3 X   = glm::normalize(glm::cross(ref, Z));
    glm::vec3 Y   = glm::cross(Z, X);

    for (int i = 0; i < SEGS; ++i) {
        float a0 = 2.f*glm::pi<float>() * float(i)   / float(SEGS);
        float a1 = 2.f*glm::pi<float>() * float(i+1) / float(SEGS);
        glm::vec3 r0 = X*std::cos(a0) + Y*std::sin(a0);
        glm::vec3 r1 = X*std::cos(a1) + Y*std::sin(a1);
        glm::vec3 p0 = base_center + r0 * radius;
        glm::vec3 p1 = base_center + r1 * radius;

        // Side triangle
        glm::vec3 sn = glm::normalize(glm::cross(p1-apex, p0-apex));
        solidBuffer_.push_back({apex, sn});
        solidBuffer_.push_back({p1,   sn});
        solidBuffer_.push_back({p0,   sn});

        // Base cap
        solidBuffer_.push_back({base_center, -Z});
        solidBuffer_.push_back({p0,          -Z});
        solidBuffer_.push_back({p1,          -Z});
    }
}

void Renderer3D::appendCylinder(glm::vec3 center, glm::vec3 axis,
                                  float radius, float half_height) {
    constexpr int SEGS = 18;
    float len = glm::length(axis);
    if (len < 1e-7f) return;
    glm::vec3 Z   = axis / len;
    glm::vec3 ref = (std::abs(Z.y) < 0.9f) ? glm::vec3(0,1,0) : glm::vec3(1,0,0);
    glm::vec3 X   = glm::normalize(glm::cross(ref, Z));
    glm::vec3 Y   = glm::cross(Z, X);

    glm::vec3 top    = center + Z * half_height;
    glm::vec3 bottom = center - Z * half_height;

    for (int i = 0; i < SEGS; ++i) {
        float a0 = 2.f*glm::pi<float>() * float(i)   / float(SEGS);
        float a1 = 2.f*glm::pi<float>() * float(i+1) / float(SEGS);
        glm::vec3 r0 = X*std::cos(a0) + Y*std::sin(a0);
        glm::vec3 r1 = X*std::cos(a1) + Y*std::sin(a1);

        glm::vec3 t0 = top    + r0*radius, t1 = top    + r1*radius;
        glm::vec3 b0 = bottom + r0*radius, b1 = bottom + r1*radius;

        // Side quad (2 triangles)
        solidBuffer_.push_back({t0, r0}); solidBuffer_.push_back({b0, r0}); solidBuffer_.push_back({t1, r1});
        solidBuffer_.push_back({t1, r1}); solidBuffer_.push_back({b0, r0}); solidBuffer_.push_back({b1, r1});

        // Top cap
        solidBuffer_.push_back({top, Z}); solidBuffer_.push_back({t0, Z}); solidBuffer_.push_back({t1, Z});
        // Bottom cap
        solidBuffer_.push_back({bottom,-Z}); solidBuffer_.push_back({b1,-Z}); solidBuffer_.push_back({b0,-Z});
    }
}

void Renderer3D::appendBox(glm::vec3 c, glm::vec3 he, const glm::mat3& rot) {
    // rot is stored col-major (glm::mat3 convention): rot[col][row]
    // Protocol sends row-major 3×3; GVizServer built it as glm::mat3(row0, row1, row2)
    // so rot[col][row] == R_row_col — same as rot.col(col).row(row)
    glm::vec3 ax = glm::vec3(rot[0][0], rot[0][1], rot[0][2]) * he.x;
    glm::vec3 ay = glm::vec3(rot[1][0], rot[1][1], rot[1][2]) * he.y;
    glm::vec3 az = glm::vec3(rot[2][0], rot[2][1], rot[2][2]) * he.z;

    // 8 corner positions
    glm::vec3 v[8] = {
        c-ax-ay-az, c+ax-ay-az, c+ax+ay-az, c-ax+ay-az,  // bottom face
        c-ax-ay+az, c+ax-ay+az, c+ax+ay+az, c-ax+ay+az   // top face
    };
    // 6 faces (each = 2 triangles, outward normal)
    auto face = [&](int a,int b,int cc,int d) {
        glm::vec3 n = glm::normalize(glm::cross(v[b]-v[a], v[cc]-v[a]));
        solidBuffer_.push_back({v[a],n}); solidBuffer_.push_back({v[b],n}); solidBuffer_.push_back({v[cc],n});
        solidBuffer_.push_back({v[a],n}); solidBuffer_.push_back({v[cc],n}); solidBuffer_.push_back({v[d],n});
    };
    face(0,1,2,3); // -Z
    face(4,7,6,5); // +Z
    face(0,4,5,1); // -Y
    face(3,2,6,7); // +Y
    face(0,3,7,4); // -X
    face(1,5,6,2); // +X
}

void Renderer3D::appendArrowHead(glm::vec3 tip, glm::vec3 dir, float head_len, float head_radius) {
    glm::vec3 base = tip - dir * head_len;
    appendCone(tip, base, head_radius);
}

} // namespace gtsam_viz
