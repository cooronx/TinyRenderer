#include "camera.h"
#include "ishader.h"
#include "model.h"
#include "tgaimage.h"
#include "tinyrenderer.h"
#include "window_config.h"
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <iostream>
#include <memory>

using namespace renderer;
using Eigen::Vector4f;

Vector3f kLightDir { -1, -1, -2 };
Vector3f target { 0, 0, 0 };
Vector3f camera_pos { 0, 1, 1 };

std::unique_ptr<Model> model { new Model("../obj/diablo3_pose.obj") };
std::array<std::array<float, kWidth>, kHeight> shadow_buffer {};
class ShadowShader : public IShader {
private:
    Vector3f varing_intensity_;
    Vector3f world_z_;
    MatrixXf varing_uv_ { 2, 3 }; // 三角形顶点的纹理坐标
    MatrixXf varing_normal_ { 3, 3 }; // 三角形顶点的法线坐标
    MatrixXf vertex_ndc_ { 3, 3 };

private:
    Matrix4f projection_;
    Matrix4f model_;
    Matrix4f view_;
    Matrix4f viewport_;
    Matrix3f normal_matrix_;

public:
    ShadowShader(const Matrix4f& projection, const Matrix4f& model, const Matrix4f& view, const Matrix4f& viewport)
        : projection_(projection)
        , model_(model)
        , view_(view)
        , viewport_(viewport)
    {
        normal_matrix_ = (((view_ * model_).inverse()).transpose()).block(0, 0, 3, 3);

        //  normal_matrix_ = normal_matrix_.block(0, 0, 3, 3);
    }

public:
    Vector3f GetWorldZ()
    {
        return world_z_;
    }
    Vector4f Vertex(int face_index, int vertex_num) override
    {

        // 计算三角形每个顶点的光照强度
        varing_intensity_[vertex_num] = std::max(0.f, model->GetVertexNorm(face_index, vertex_num).dot(kLightDir));
        // 计算三角形每个顶点的uv值
        varing_uv_.col(vertex_num) = model->GetUVByIndex(face_index, vertex_num);
        // 计算三角形每个顶点的法线向量
        varing_normal_.col(vertex_num) = normal_matrix_ * model->GetVertexNorm(face_index, vertex_num);

        auto vertex = model->GetVertPosByIndex(face_index, vertex_num);
        auto gl_vertex = Vector4f { vertex.x(), vertex.y(), vertex.z(), 1.0f };
        // 计算齐次空间中的坐标
        gl_vertex = viewport_ * projection_ * view_ * model_ * gl_vertex;

        // NDC坐标系中的坐标
        vertex_ndc_.col(vertex_num) = (gl_vertex / gl_vertex.w()).head<3>();

        // 透视矫正插值
        world_z_[vertex_num] = gl_vertex.z() - camera_pos.z();
        return gl_vertex;
    }
    bool Fragment(Eigen::Vector3f barycentric, TGAColor& color) override
    {

        Vector3f pos = vertex_ndc_ * barycentric;
        color = TGAColor(255, 255, 255, 255) * (2.f * pos[2] / kDepth);
        return false;
    }
};

class BlinnPhongShader : public IShader {
private:
    Vector3f varing_intensity_;
    Vector3f world_z_;
    MatrixXf light_pos_ { 3, 3 };
    MatrixXf varing_uv_ { 2, 3 }; // 三角形顶点的纹理坐标
    MatrixXf varing_normal_ { 3, 3 }; // 三角形顶点的法线坐标
    MatrixXf vertex_ndc_ { 3, 3 };
    // std::array<std::array<float, kWidth>, kHeight> shadow_buffer_ {};

private:
    Matrix4f projection_;
    Matrix4f model_;
    Matrix4f view_;
    Matrix4f viewport_;
    Matrix3f normal_matrix_;
    Matrix4f light_matrix_;

public:
    BlinnPhongShader(const Matrix4f& projection, const Matrix4f& model, const Matrix4f& view, const Matrix4f& viewport, const Matrix4f& light_matrix)
        : projection_(projection)
        , model_(model)
        , view_(view)
        , viewport_(viewport)
        , light_matrix_(light_matrix)
    {
        normal_matrix_ = (((view_ * model_).inverse()).transpose()).block(0, 0, 3, 3);
    }

public:
    Vector3f GetWorldZ()
    {
        return world_z_;
    }
    Vector4f Vertex(int face_index, int vertex_num) override
    {

        // 计算三角形每个顶点的光照强度
        varing_intensity_[vertex_num] = std::max(0.f, model->GetVertexNorm(face_index, vertex_num).dot(kLightDir));
        // 计算三角形每个顶点的uv值
        varing_uv_.col(vertex_num) = model->GetUVByIndex(face_index, vertex_num);
        // 计算三角形每个顶点的法线向量
        varing_normal_.col(vertex_num) = normal_matrix_ * model->GetVertexNorm(face_index, vertex_num);

        auto vertex = model->GetVertPosByIndex(face_index, vertex_num);
        auto gl_vertex = Vector4f { vertex.x(), vertex.y(), vertex.z(), 1.0f };
        // 光空间坐标
        Vector4f temp = light_matrix_ * gl_vertex;
        light_pos_.col(vertex_num) = TinyRenderer::HomoDivision(temp);
        // 计算齐次空间中的坐标
        gl_vertex = viewport_ * projection_ * view_ * model_ * gl_vertex;
        // NDC坐标系中的坐标
        vertex_ndc_.col(vertex_num) = (gl_vertex / gl_vertex.w()).head<3>();
        // 透视矫正插值
        world_z_[vertex_num] = gl_vertex.z() - camera_pos.z();

        return gl_vertex;
    }
    bool Fragment(Eigen::Vector3f barycentric, TGAColor& color) override
    {
        // 转换到光照坐标系中进行转换
        auto temp = vertex_ndc_ * barycentric;
        Vector4f interpolated_light_pos { temp.x(), temp.y(), temp.z(), 1.0f };
        interpolated_light_pos = (light_matrix_ * (viewport_ * projection_ * view_ * model_).inverse()) * interpolated_light_pos;
        Vector3f li_pos = (interpolated_light_pos / interpolated_light_pos.w()).head<3>();

        Vector2f interpolated_uv = varing_uv_ * barycentric;
        Vector3f interpolated_normal = varing_normal_ * barycentric;
        interpolated_normal.normalize();

        // 三角形两条边的向量
        Vector3f P1 = vertex_ndc_.col(1) - vertex_ndc_.col(0);
        Vector3f P2 = vertex_ndc_.col(2) - vertex_ndc_.col(0);

        Vector3f uv1 = Vector3f(varing_uv_(0, 1) - varing_uv_(0, 0), varing_uv_(1, 1) - varing_uv_(1, 0), 0);
        Vector3f uv2 = Vector3f(varing_uv_(0, 2) - varing_uv_(0, 0), varing_uv_(1, 2) - varing_uv_(1, 0), 0);

        Vector3f T = (P1 * uv2[1] - P2 * uv1[1]) / (uv1[0] * uv2[1] - uv2[0] * uv1[1]);
        Vector3f B = (P2 * uv1[0] - P1 * uv2[0]) / (uv1[0] * uv2[1] - uv2[0] * uv1[1]);

        // 施密特正交化
        Vector3f t_ = T - (T.dot(interpolated_normal)) * interpolated_normal;
        t_.normalize();
        Vector3f b_ = B - B.dot(interpolated_normal) * interpolated_normal - B.dot(t_) * t_;
        b_.normalize();

        // TBN矩阵
        Eigen::Matrix3f M_tbn;
        M_tbn.col(0) = t_;
        M_tbn.col(1) = b_;
        M_tbn.col(2) = interpolated_normal;

        // 切线空间法线变换到世界空间
        Vector3f n = M_tbn * model->Normal(interpolated_uv);
        n.normalize();
        // 入射光向量
        Vector3f l = (kLightDir.homogeneous()).head<3>();
        l.normalize();
        float diff = std::max(n.dot(l), 0.f);
        // 视线向量
        Vector3f v = ((camera_pos.homogeneous())).head<3>();
        v.normalize();
        // 半程向量
        Vector3f h = (v + l).normalized();
        float spec = pow(std::max(h.dot(n), 0.f), model->Spec(interpolated_uv));
        float shadow = 0.3f + 0.7f * (shadow_buffer[li_pos[0]][li_pos[1]] < li_pos[2] + 45.0f);
        float intensity = diff + 0.6f * spec;
        TGAColor c = model->Diffuse(interpolated_uv);
        for (int i = 0; i < 3; i++)
            color.raw[i] = std::min<float>(c.raw[i] * shadow * intensity, 255);

        return false;
    }

private:
    // 计算发射光向量
    Vector4f Reflect(Vector4f in, Vector4f normal)
    {
        Vector4f reflect_vec = (in.dot(normal) * normal * 2.0f - in);
        reflect_vec.normalize();
        return reflect_vec;
    }
};

static void init()
{
    // 定向光的方向向量记得归一化
    // 取反是为了计算点乘
    kLightDir *= -1;
    kLightDir.normalize();
}

int main(int argc, char** argv)
{
    init(); // 完成各种初始化
    std::array<Vector4f, 3> world_pos {};
    std::array<Vector3f, 3> screen_pos {};
    Matrix4f light_matrix {};

    // 渲染阴影贴图
    // 阴影图
    TGAImage shadow_image(kHeight, kWidth, TGAImage::GRAYSCALE);

    // 初始化相机
    auto shadow_camera = std::make_unique<Camera>(kLightDir, target);
    // 初始化渲染器
    auto shadow_renderer = std::make_unique<TinyRenderer>();
    // 这里要将视口左下脚略微上移动，不然会出现负的坐标值
    Matrix4f view_port = TinyRenderer::ViewPort(kWidth / 8, kHeight / 8, kWidth * 3 / 4, kHeight * 3 / 4);
    // 这里只需要一个正交投影矩阵即可
    Matrix4f projection = Matrix4f::Identity();
    projection(3, 2) = 0.f;
    // 初始化look_at矩阵
    auto look_at = shadow_camera->LookAt();
    // shader实现
    auto shadow_shader = ShadowShader(projection, Matrix4f::Identity(), look_at, view_port);

    for (int i = 0; i < model->GetFaceSize(); ++i) {
        for (int j = 0; j < 3; ++j) {
            world_pos[j] = shadow_shader.Vertex(i, j);
            screen_pos[j] = TinyRenderer::HomoDivision(world_pos[j]);
        }
        shadow_renderer->DrawTriangle(screen_pos, shadow_shader.GetWorldZ(), shadow_shader, shadow_image, true);
    }
    shadow_image.flip_vertically(); // 中心点在图片的左下角
    shadow_image.write_tga_file("../output/shadow.tga");
    // byd光照方向的矩阵写错了，找半天才找出来
    light_matrix = view_port * projection * look_at;
    shadow_buffer = shadow_renderer->z_buffer;

    // output
    TGAImage image(kHeight, kWidth, TGAImage::RGB);

    // 初始化相机
    auto camera = std::make_unique<Camera>(camera_pos, target);
    // 初始化渲染器
    auto renderer = std::make_unique<TinyRenderer>();
    // 这里要将视口左下脚略微上移动，不然会出现负的坐标值
    view_port = TinyRenderer::ViewPort(kWidth / 8, kHeight / 8, kWidth * 3 / 4, kHeight * 3 / 4);
    // 这里只需要一个正交投影矩阵即可
    projection = TinyRenderer::PerspectiveProjection(90.0f, kWidth * 1.0f / kHeight, 0.1f, 200.f);
    // 初始化look_at矩阵
    look_at = camera->LookAt();
    // shader实现
    auto shader = BlinnPhongShader(projection, Matrix4f::Identity(), look_at, view_port, light_matrix);

    for (int i = 0; i < model->GetFaceSize(); ++i) {
        for (int j = 0; j < 3; ++j) {

            world_pos[j] = shader.Vertex(i, j);
            screen_pos[j] = TinyRenderer::HomoDivision(world_pos[j]);
        }
        renderer->DrawTriangle(screen_pos, shader.GetWorldZ(), shader, image, false);
    }
    image.flip_vertically(); // 中心点在图片的左下角
    image.write_tga_file("../output/output.tga");
    return 0;
}