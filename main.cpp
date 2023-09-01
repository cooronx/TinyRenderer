
#include "tgaimage.h"
#include "window_config.h"
#include "camera.h"
#include "model.h"
#include <memory>
#include <array>
#include <Eigen/Dense>
#include <iostream>


using namespace renderer;
using Eigen::Vector4f;

const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0,255,0,255);
Vector3f target{0,0,0};
Vector3f camera_pos{0,0,2};

//我们这里默认近平面=0，远平面=255
const int kDepth = 255;

std::array<float,kPixelsCount> z_buffer{};

std::unique_ptr<Model> model{new Model("../obj/african_head.obj")};





/// Bresenham算法
/// \param x0
/// \param y0
/// \param x1
/// \param y1
/// \param image
/// \param color
void DrawLine(int x0, int y0, int x1, int y1, TGAImage &image, const TGAColor &color)
{
    bool steep = false;                          //直线斜率是否大于1
    if (std::abs(x0 - x1) < std::abs(y0 - y1)) { //如果斜率大于1，就对于y=x对称
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0 > x1) { //如果终点小于起点（2，3象限）
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    int dx = x1 - x0;
    int dy = std::abs(y1 - y0);
    int d0 = 2 * dy - dx;
    int increase_east = 2 * dy;
    int increase_north_east = 2 * dy - 2 * dx;
    int y = y0;
    for (int x = x0; x <= x1; ++x) {
        if (d0 < 0) {
            d0 += increase_east;
        } else {
            if (y1 > y0) {
                ++y;
            } else {
                --y;
            }
            d0 += increase_north_east;
        }
        if (steep) {
            image.set(y, x, color);
        } else {
            image.set(x, y, color);
        }
    }
}

/// Bresenham算法
/// \param p0
/// \param p1
/// \param image
/// \param color
void DrawLine(const Vector2i& p0,const Vector2i& p1,TGAImage &image, const TGAColor &color)
{
    int x0 = p0.x();
    int x1 = p1.x();
    int y0 = p0.y();
    int y1 = p1.y();
    bool steep = false;                          //直线斜率是否大于1
    if (std::abs(x0 - x1) < std::abs(y0 - y1)) { //如果斜率大于1，就对于y=x对称
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0 > x1) { //如果终点小于起点（2，3象限）
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    int dx = x1 - x0;
    int dy = std::abs(y1 - y0);
    int d0 = 2 * dy - dx;
    int increase_east = 2 * dy;
    int increase_north_east = 2 * dy - 2 * dx;
    int y = y0;
    for (int x = x0; x <= x1; ++x) {
        if (d0 < 0) {
            d0 += increase_east;
        } else {
            if (y1 > y0) {
                ++y;
            } else {
                --y;
            }
            d0 += increase_north_east;
        }
        if (steep) {
            image.set(y, x, color);
        } else {
            image.set(x, y, color);
        }
    }
}

/// 求解P点的重心坐标
/// \param pts 三角形的三个顶点
/// \param P
/// \return 该点的重心坐标
Vector3f barycentric(const std::array<Vector3f,3> &pts,const Vector2i& P) {
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
    return {alpha, beta, gamma};
}



/// 光栅化绘制三角形
/// \param pts 三角形三个顶点的坐标
/// \param uv_pos 三角形三个顶点的纹理坐标
/// \param z_buffer z缓存
/// \param intensity 光照强度
/// \param image 图片
void DrawTriangle(const std::array<Vector3f, 3>& pts, const std::array<Vector2f, 3>& uv_pos, std::array<float, kPixelsCount>& z_buffer,
                  float intensity, TGAImage& image) {
    Vector2f bounding_box_min{ std::numeric_limits<float>::max(),std::numeric_limits<float>::max() };
    Vector2f bounding_box_max{};
    Vector2f clamp{ static_cast<float>(image.get_width() - 1),static_cast<float>(image.get_height() - 1) };//设置裁切大小，防止三角形超出了图片

    //比较求出三角形的包围箱
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
            if (bc_screen.x() < -0.01f || bc_screen.y() < -0.01f || bc_screen.z() < -0.01f) continue;
            //用重心坐标插值计算该点的深度
            float interpolated_z = bc_screen.x() * pts[0].z() + bc_screen.y() * pts[1].z() + bc_screen.z() * pts[2].z();
            //用重心坐标插值计算该点的纹理坐标
            Vector2f interpolated_uv_pos = uv_pos[0] * bc_screen.x() + uv_pos[1] * bc_screen.y()
                    + uv_pos[2] * bc_screen.z();

            if (interpolated_z > z_buffer[x + y * kWidth]) {
                z_buffer[x + y * kWidth] = interpolated_z;
                TGAColor color = model->Diffuse(interpolated_uv_pos);

                image.set(x, y, { static_cast<unsigned char>(color.r * intensity),static_cast<unsigned char>(color.g * intensity),static_cast<unsigned char>(color.b * intensity),255 });
            }
        }
    }
}



/// 世界坐标转换为屏幕坐标
/// \param world_pos 世界坐标
/// \return 屏幕坐标
Vector3f WorldToScreen(const Vector3f& world_pos){
    return Vector3f {static_cast<float>((world_pos.x() + 1.0)*kWidth/2.0),static_cast<float>((world_pos.y() + 1.0)*kHeight/2.0),world_pos.z()};
}

/// 除掉齐次坐标的w分量(齐次除法)
/// \param m
/// \return
Vector3f HomoDivision(const Eigen::Matrix4Xf m) {
    return {m(0,0) / m(3,0), m(1,0)/m(3,0), m(2,0)/m(3,0)};
}



/// 透视投影
/// \return 投影矩阵
Matrix4f PerspectiveProjection(float eye_fov, float aspect_ratio,
                               float zNear, float zFar){
    eye_fov = eye_fov / 180 * EIGEN_PI;
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f aspect_fovY;
    float ty = -1.0f / tan(eye_fov / 2.0f);
    aspect_fovY << (ty / aspect_ratio), 0, 0, 0,
            0, ty, 0, 0,
            0, 0, (zNear+zFar)/(zNear-zFar), (-2*zNear*zFar)/(zFar-zNear),
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
Matrix4f ViewPort(int x, int y, int w, int h) {
    Matrix4f m = Matrix4f::Identity();
    m(0,3) = x+w/2.f;
    m(1,3) = y+h/2.f;
    m(2,3) = kDepth/2.f;

    m(0,0) = w/2.f;
    m(1,1) = h/2.f;
    m(2,2) = kDepth/2.f;
    return m;
}

int main(int argc, char **argv)
{

    TGAImage image(kHeight, kWidth, TGAImage::RGB);

    //初始化相机
    auto camera = std::make_unique<Camera>(camera_pos,target);
    auto look_at = camera->LookAt();


    //初始化z_buffer
    std::fill(z_buffer.begin(),z_buffer.end(),std::numeric_limits<float>::min());

    std::array<Vector3f,3> screen_pos;
    std::array<Vector3f,3> world_pos;

    //这里要将视口左下脚略微上移动，不然会出现负的坐标值
    Matrix4f view_port = ViewPort(kWidth / 8, kHeight / 8, kWidth * 3 / 4, kHeight * 3 / 4);

    auto per = PerspectiveProjection(80.0f,1.0*kWidth/kHeight,  0.1f,200.0f);
    //std::cout<<per;
    for(int i = 0;i<model->GetFaceSize();++i){
        auto face = model->GetVertexIndex(i);
        for(int j = 0;j<3;++j){
            auto vertex = model->GetVertByIndex(face[j]);
            Vector4f homo_vertex = {vertex.x(),vertex.y(),vertex.z(),1.0f};
            screen_pos[j] = HomoDivision(view_port *
                    per *
                    look_at *
                    homo_vertex);
            world_pos[j] = vertex;
        }
        Vector3f norm_vec = (world_pos[2] - world_pos[0]).cross(world_pos[1] - world_pos[0]);
        norm_vec.normalize();
        Vector3f dir = Vector3f(0,0,-1);
        float intensity = norm_vec.dot(dir);
        if(intensity > 0){
            std::array <Vector2f,3>uv_pos;
            for(int k = 0;k<3;++k) uv_pos[k] = model->GetUVByIndex(i,k);
            DrawTriangle(screen_pos,uv_pos,z_buffer,intensity,image);
        }
    }
    image.flip_vertically(); // 中心点在图片的左下角
    image.write_tga_file("../output/output.tga");

    //画出Z_Buffer的图像
    TGAImage z_buffer_image(kWidth, kHeight, TGAImage::GRAYSCALE);
    for (int i=0; i<kWidth; i++) {
        for (int j=0; j<kHeight; j++) {
            z_buffer_image.set(i, j, TGAColor(z_buffer[i + j * kWidth], 1));
        }
    }
    z_buffer_image.flip_vertically();
    z_buffer_image.write_tga_file("../output/zbuffer.tga");

    return 0;
}