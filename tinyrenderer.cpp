//
// Created by cooronx on 2023/8/31.
//

#include "tinyrenderer.h"
#include <iostream>

/// 透视投影
/// \return 投影矩阵
Matrix4f renderer::TinyRenderer::PerspectiveProjection(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    eye_fov = eye_fov / 180 * EIGEN_PI;
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f aspect_fovY;
    float ty = -1.0f / tan(eye_fov / 2.0f);
    aspect_fovY << (ty / aspect_ratio), 0, 0, 0,
        0, ty, 0, 0,
        0, 0, (zNear + zFar) / (zNear - zFar), (-2 * zNear * zFar) / (zFar - zNear),
        0, 0, 1, 0;
    projection = aspect_fovY * projection;
    return projection;
}

/// 计算视口矩阵
/// \param x
/// \param y
/// \param w
/// \param h
/// \return 视口矩阵
Matrix4f renderer::TinyRenderer::ViewPort(int x, int y, int w, int h)
{
    Matrix4f m = Matrix4f::Identity();
    m(0, 3) = x + w / 2.f;
    m(1, 3) = y + h / 2.f;
    m(2, 3) = kDepth / 2.f;

    m(0, 0) = w / 2.f;
    m(1, 1) = h / 2.f;
    m(2, 2) = kDepth / 2.f;
    return m;
}

renderer::TinyRenderer::TinyRenderer()
{
    for (auto& row : z_buffer) {
        for (auto& cell : row) {
            cell = -std::numeric_limits<float>::max();
        }
    }
}

void renderer::TinyRenderer::DrawTriangle(const std::array<Vector3f, 3>& pts, const Vector3f& world_z, IShader& shader, TGAImage& image, bool isShadow)
{

    Vector2f bounding_box_min { std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
    Vector2f bounding_box_max {};
    Vector2f clamp { static_cast<float>(image.get_width() - 1), static_cast<float>(image.get_height() - 1) }; // 设置裁切大小，防止三角形超出了图片

    // 比较求出三角形的包围箱
    for (const auto& point : pts) {
        bounding_box_min.x() = std::min(bounding_box_min.x(), point.x());
        bounding_box_min.y() = std::min(bounding_box_min.y(), point.y());

        bounding_box_max.x() = std::min(clamp.x(), std::max(bounding_box_max.x(), point.x()));
        bounding_box_max.y() = std::min(clamp.y(), std::max(bounding_box_max.y(), point.y()));
    }

    int min_x = floor(bounding_box_min.x());
    int max_x = ceil(bounding_box_max.x());
    int min_y = floor(bounding_box_min.y());
    int max_y = ceil(bounding_box_max.y());

    for (int x = min_x; x <= max_x; ++x) {
        for (int y = min_y; y <= max_y; ++y) {
            Vector3f bc_screen = barycentric(pts, { x, y });
            Vector3f bc_revised {}; // 进行透视矫正插值
            for (int i = 0; i < 3; ++i) {
                // 求α，β，γ
                bc_revised[i] = bc_screen[i] / world_z[i]; // world_pts[i][2];
            }
            float Z_n = 1. / (bc_revised[0] + bc_revised[1] + bc_revised[2]);
            for (int i = 0; i < 3; ++i) {
                // 求正确透视下插值的系数
                bc_revised[i] *= Z_n;
            }
            if (bc_screen.x() < -0.01f || bc_screen.y() < -0.01f || bc_screen.z() < -0.01f)
                continue;
            float interpolated_z_value = bc_screen.x() * pts[0].z() + bc_screen.y() * pts[1].z() + bc_screen.z() * pts[2].z();
            if (interpolated_z_value > z_buffer[x][y]) {
                z_buffer[x][y] = interpolated_z_value;

                TGAColor color {};

                if (isShadow) {
                    shader.Fragment(bc_screen, color);
                } else {
                    shader.Fragment(bc_revised, color);
                }
                image.set(x, y, color);
            }
        }
    }
}

Vector3f renderer::TinyRenderer::barycentric(const std::array<Vector3f, 3>& pts, const Vector2i& P)
{
    float xa = pts[0].x();
    float ya = pts[0].y();
    float xb = pts[1].x();
    float yb = pts[1].y();
    float xc = pts[2].x();
    float yc = pts[2].y();
    auto x = static_cast<float>(P.x());
    auto y = static_cast<float>(P.y());
    float gamma = static_cast<float>((ya - yb) * x + (xb - xa) * y + xa * yb - xb * ya) / static_cast<float>((ya - yb) * xc + (xb - xa) * yc + xa * yb - xb * ya);
    float beta = static_cast<float>((ya - yc) * x + (xc - xa) * y + xa * yc - xc * ya) / static_cast<float>((ya - yc) * xb + (xc - xa) * yb + xa * yc - xc * ya);
    float alpha = 1 - gamma - beta;
    return { alpha, beta, gamma };
}

Vector3f renderer::TinyRenderer::HomoDivision(Vector4f vec)
{
    return Eigen::Vector3f { vec[0] / vec[3], vec[1] / vec[3], vec[2] / vec[3] };
}

float renderer::TinyRenderer::GetZValue(int i, int j)
{
    return z_buffer[i][j];
}
