#include "Renderer3D.h"
#include <glm/gtc/type_ptr.hpp>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <cstring>
#include <algorithm>

namespace gtsam_viz {

// ── GLSL ──────────────────────────────────────────────────────────────────────
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
    yaw   += dYaw;
    pitch  = glm::clamp(pitch + dPitch, -89.f, 89.f);
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
    // Move target in the horizontal (XZ) plane aligned to camera yaw
    float yR = glm::radians(yaw);
    glm::vec3 fwd  = glm::normalize(glm::vec3(std::cos(yR), 0, std::sin(yR)));
    glm::vec3 rght = glm::normalize(glm::cross(fwd, glm::vec3(0,1,0)));
    float speed = distance * 0.8f;
    target += fwd * forward * speed + rght * right_ * speed;
}

// ── Renderer3D ────────────────────────────────────────────────────────────────
Renderer3D::Renderer3D()  = default;
Renderer3D::~Renderer3D() { shutdown(); }

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
void Renderer3D::setUniformMat4(GLuint prog, const char* n, const glm::mat4& m) {
    glUniformMatrix4fv(glGetUniformLocation(prog,n),1,GL_FALSE,glm::value_ptr(m));
}
void Renderer3D::setUniformVec4(GLuint prog, const char* n, const glm::vec4& v) {
    glUniform4fv(glGetUniformLocation(prog,n),1,glm::value_ptr(v));
}
void Renderer3D::setUniformVec3(GLuint prog, const char* n, const glm::vec3& v) {
    glUniform3fv(glGetUniformLocation(prog,n),1,glm::value_ptr(v));
}
void Renderer3D::setUniformFloat(GLuint prog, const char* n, float v) {
    glUniform1f(glGetUniformLocation(prog,n), v);
}

bool Renderer3D::init() {
    if (initialized_) return true;
    try {
        buildShaders();
        buildGridMesh();
        buildAxisMesh();
        buildSphereMesh();

        glGenVertexArrays(1, &lineVao_); glGenBuffers(1, &lineVbo_);
        glBindVertexArray(lineVao_);
        glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(LineVertex),(void*)offsetof(LineVertex,pos));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1,4,GL_FLOAT,GL_FALSE,sizeof(LineVertex),(void*)offsetof(LineVertex,color));
        glBindVertexArray(0);

        initialized_ = true;
        return true;
    } catch (...) { return false; }
}

void Renderer3D::shutdown() {
    if (!initialized_) return;
    if (fbo_)     glDeleteFramebuffers(1,  &fbo_);
    if (colorTex_)glDeleteTextures(1,      &colorTex_);
    if (depthRbo_)glDeleteRenderbuffers(1, &depthRbo_);
    glDeleteVertexArrays(1,&gridVao_);   glDeleteBuffers(1,&gridVbo_);
    glDeleteVertexArrays(1,&axisVao_);   glDeleteBuffers(1,&axisVbo_);
    glDeleteVertexArrays(1,&sphereVao_); glDeleteBuffers(1,&sphereVbo_);
    glDeleteBuffers(1,&sphereEbo_);
    glDeleteVertexArrays(1,&lineVao_);   glDeleteBuffers(1,&lineVbo_);
    glDeleteProgram(phongProg_);
    glDeleteProgram(flatProg_);
    glDeleteProgram(lineProg_);
    initialized_ = false;
}

void Renderer3D::resize(int w, int h) {
    if (w == fboW_ && h == fboH_) return;
    fboW_ = w; fboH_ = h;
    if (fbo_)     glDeleteFramebuffers(1,  &fbo_);
    if (colorTex_)glDeleteTextures(1,      &colorTex_);
    if (depthRbo_)glDeleteRenderbuffers(1, &depthRbo_);

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

GLuint Renderer3D::render(const FactorGraphState& state) {
    if (!initialized_ || fboW_==0 || fboH_==0) return 0;

    // Rebuild grid if params changed
    if (gridHalf != lastGridHalf_ || gridStep != lastGridStep_)
        rebuildGrid();

    // Follow mode: keep target on the latest pose
    if (followMode && !state.variables().empty())
        cam_.target = state.variables().back().position3d;

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glViewport(0,0,fboW_,fboH_);
    glClearColor(bgColor.r,bgColor.g,bgColor.b,bgColor.a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    float     aspect = float(fboW_)/float(fboH_);
    glm::mat4 VP     = cam_.projection(aspect) * cam_.view();

    drawGrid();

    lineBuffer_.clear();

    auto& vars    = state.variables();
    auto& factors = state.factors();

    // Trajectory edges
    if (showTrajectory) {
        for (auto& fn : factors) {
            if (fn.keys.size() < 2) continue;
            for (size_t i = 0; i+1 < fn.keys.size(); ++i) {
                glm::vec3 pa{}, pb{};
                bool fa=false, fb=false;
                for (auto& vn : vars) {
                    if (vn.key==fn.keys[i])   { pa=vn.position3d; fa=true; }
                    if (vn.key==fn.keys[i+1]) { pb=vn.position3d; fb=true; }
                }
                if (fa && fb) {
                    float t   = glm::clamp((float)fn.error * 2.f, 0.f, 1.f);
                    glm::vec4 col = glm::mix(glm::vec4{0.3f,0.9f,0.5f,0.9f},
                                             glm::vec4{1.f,0.3f,0.2f,0.9f}, t);
                    lineBuffer_.push_back({pa, col});
                    lineBuffer_.push_back({pb, col});
                }
            }
        }
    }

    // Poses
    for (auto& vn : vars) {
        bool isPose = vn.type==VariableType::Pose2 || vn.type==VariableType::Pose3;
        if (showAxes) drawPoseAxes(vn.transform, axisLength);
        drawSphere(vn.position3d, nodeSize, nodeSize, nodeSize,
                   glm::mat4(1.f),
                   isPose ? glm::vec4{0.3f,0.65f,1.f,1.f}
                          : glm::vec4{0.3f,1.f,0.6f,1.f});
    }

    // Covariance ellipsoids
    if (showCovEllipsoids) {
        for (auto& vn : vars) {
            auto cov = state.marginalCovariance(vn.key);
            if (!cov) continue;
            const gtsam::Matrix& C = *cov;
            // GTSAM Pose3 tangent: [rot(0-2), trans(3-5)]
            // Use the 3x3 position sub-block (indices 3-5) for the ellipsoid
            int off = (C.rows() == 6) ? 3 : 0; // Pose3=6, Point3=3
            float sx = std::sqrt(std::max(0.0, C(off+0, off+0))) * covScale;
            float sy = std::sqrt(std::max(0.0, C(off+1, off+1))) * covScale;
            float sz = std::sqrt(std::max(0.0, C(off+2, off+2))) * covScale;
            // Rotation from pose transform (orientation of ellipsoid)
            glm::mat4 rot = glm::mat4(
                glm::vec4(vn.transform[0].x, vn.transform[0].y, vn.transform[0].z, 0),
                glm::vec4(vn.transform[1].x, vn.transform[1].y, vn.transform[1].z, 0),
                glm::vec4(vn.transform[2].x, vn.transform[2].y, vn.transform[2].z, 0),
                glm::vec4(0,0,0,1));
            drawSphere(vn.position3d, sx, sy, sz, rot,
                       {0.9f, 0.7f, 0.2f, 0.25f});
        }
    }

    // Flush line buffer
    if (!lineBuffer_.empty()) {
        glUseProgram(lineProg_);
        setUniformMat4(lineProg_, "uVP", VP);
        glBindVertexArray(lineVao_);
        glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
        glBufferData(GL_ARRAY_BUFFER,
                     lineBuffer_.size()*sizeof(LineVertex),
                     lineBuffer_.data(), GL_DYNAMIC_DRAW);
        glLineWidth(2.f);
        glDrawArrays(GL_LINES, 0, (GLsizei)lineBuffer_.size());
        glBindVertexArray(0);
    }

    glBindFramebuffer(GL_FRAMEBUFFER,0);
    return colorTex_;
}

// ── Mesh builders ─────────────────────────────────────────────────────────────
void Renderer3D::buildShaders() {
    phongProg_ = linkProgram(compileShader(GL_VERTEX_SHADER, PHONG_VERT),
                             compileShader(GL_FRAGMENT_SHADER,PHONG_FRAG));
    flatProg_  = linkProgram(compileShader(GL_VERTEX_SHADER, FLAT_VERT),
                             compileShader(GL_FRAGMENT_SHADER,FLAT_FRAG));
    lineProg_  = linkProgram(compileShader(GL_VERTEX_SHADER, LINE_VERT),
                             compileShader(GL_FRAGMENT_SHADER,LINE_FRAG));
}

void Renderer3D::buildGridMesh() {
    lastGridHalf_ = gridHalf;
    lastGridStep_ = gridStep;

    std::vector<float> verts;
    verts.reserve((gridHalf*2+1)*4*3);
    for (int i = -gridHalf; i <= gridHalf; ++i) {
        float fi = float(i) * gridStep;
        float fh = float(gridHalf) * gridStep;
        verts.insert(verts.end(), { fi,  0.f, -fh });
        verts.insert(verts.end(), { fi,  0.f,  fh });
        verts.insert(verts.end(), {-fh,  0.f,  fi });
        verts.insert(verts.end(), { fh,  0.f,  fi });
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
    if (!initialized_) return;
    buildGridMesh();
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
            verts.insert(verts.end(),{x,y,z,  x,y,z});
        }
    }
    for (int i=0;i<stacks;++i)
        for (int j=0;j<slices;++j) {
            int a=i*(slices+1)+j, b=a+slices+1;
            idx.insert(idx.end(),{(unsigned)a,(unsigned)b,(unsigned)(a+1),
                                  (unsigned)(a+1),(unsigned)b,(unsigned)(b+1)});
        }
    sphereIdxCount_ = (int)idx.size();
    glGenVertexArrays(1,&sphereVao_);
    glGenBuffers(1,&sphereVbo_);
    glGenBuffers(1,&sphereEbo_);
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
void Renderer3D::drawGrid() {
    float aspect = float(fboW_)/float(fboH_);
    glm::mat4 VP = cam_.projection(aspect) * cam_.view();
    glUseProgram(flatProg_);
    setUniformMat4(flatProg_, "uMVP",   VP);
    setUniformVec4(flatProg_, "uColor", {0.20f,0.22f,0.26f,1.f});
    glBindVertexArray(gridVao_);
    glLineWidth(1.f);
    glDrawArrays(GL_LINES, 0, gridVerts_);
    glBindVertexArray(0);
}

void Renderer3D::drawPoseAxes(const glm::mat4& transform, float scale) {
    float aspect = float(fboW_)/float(fboH_);
    glm::mat4 VP = cam_.projection(aspect) * cam_.view();
    glm::mat4 M  = transform * glm::scale(glm::mat4(1.f), glm::vec3(scale));
    glLineWidth(3.f);
    glBindVertexArray(axisVao_);
    glUseProgram(flatProg_);
    setUniformMat4(flatProg_, "uMVP",   VP * M);
    setUniformVec4(flatProg_, "uColor", {1.f,0.22f,0.22f,1.f}); glDrawArrays(GL_LINES,0,2);
    setUniformVec4(flatProg_, "uColor", {0.22f,1.f,0.22f,1.f}); glDrawArrays(GL_LINES,2,2);
    setUniformVec4(flatProg_, "uColor", {0.22f,0.4f,1.f,1.f});  glDrawArrays(GL_LINES,4,2);
    glBindVertexArray(0);
}

void Renderer3D::drawSphere(glm::vec3 pos,
                             float rx, float ry, float rz,
                             const glm::mat4& rotation, glm::vec4 color) {
    float aspect = float(fboW_)/float(fboH_);
    glm::mat4 VP = cam_.projection(aspect) * cam_.view();
    glm::mat4 T  = glm::translate(glm::mat4(1.f), pos);
    glm::mat4 S  = glm::scale(glm::mat4(1.f), {rx, ry, rz});
    glm::mat4 M  = T * rotation * S;
    glm::mat3 NM = glm::mat3(glm::transpose(glm::inverse(M)));

    // Transparent ellipsoids need blending
    bool needBlend = (color.a < 0.99f);
    if (needBlend) { glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); }

    glUseProgram(phongProg_);
    setUniformMat4(phongProg_, "uMVP",   VP*M);
    setUniformMat4(phongProg_, "uModel", M);
    glUniformMatrix3fv(glGetUniformLocation(phongProg_,"uNormalMat"),1,GL_FALSE,glm::value_ptr(NM));
    setUniformVec4(phongProg_, "uColor",    color);
    setUniformVec3(phongProg_, "uLightDir", glm::normalize(glm::vec3{1,3,2}));
    glBindVertexArray(sphereVao_);
    glDrawElements(GL_TRIANGLES, sphereIdxCount_, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);

    if (needBlend) glDisable(GL_BLEND);
}

} // namespace gtsam_viz
