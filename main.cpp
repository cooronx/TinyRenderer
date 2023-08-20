#include "geometry.h"
#include "tgaimage.h"
#include "window_config.h"
#include "model.h"
#include <memory>
#include <array>

const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0,255,0,255);
const Vec3f dir{0,0,-1};


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
void DrawLine(Vec2i p0,Vec2i p1,TGAImage &image, const TGAColor &color)
{
    int x0 = p0.x;
    int x1 = p1.x;
    int y0 = p0.y;
    int y1 = p1.y;
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
Vec3f barycentric(const std::array<Vec3f,3> &pts,Vec2i P) {
    float xa = pts[0].x;
    float ya = pts[0].y;
    float xb = pts[1].x;
    float yb = pts[1].y;
    float xc = pts[2].x;
    float yc = pts[2].y;
    auto x = static_cast<float>(P.x);
    auto y = static_cast<float>(P.y);
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
void DrawTriangle(const std::array<Vec3f, 3>& pts, const std::array<Vec2i, 3>& uv_pos, std::array<float, kPixelsCount>& z_buffer,
                  float intensity, TGAImage& image) {
    Vec2f bounding_box_min{ std::numeric_limits<float>::max(),std::numeric_limits<float>::max() };
    Vec2f bounding_box_max{};
    Vec2f clamp{ static_cast<float>(image.get_width() - 1),static_cast<float>(image.get_height() - 1) };//设置裁切大小，防止三角形超出了图片

    //比较求出三角形的包围箱
    for (const auto& point : pts) {
        bounding_box_min.x = std::min(bounding_box_min.x, point.x);
        bounding_box_min.y = std::min(bounding_box_min.y, point.y);

        bounding_box_max.x = std::min(clamp.x, std::max(bounding_box_max.x, point.x));
        bounding_box_max.y = std::min(clamp.y, std::max(bounding_box_max.y, point.y));
    }

    int min_x = floor(bounding_box_min.x);
    int max_x = ceil(bounding_box_max.x);
    int min_y = floor(bounding_box_min.y);
    int max_y = ceil(bounding_box_max.y);

    for (int x = min_x; x <= max_x; ++x) {
        for (int y = min_y; y <= max_y; ++y) {
            Vec3f bc_screen = barycentric(pts, { x, y });
            if (bc_screen.x < -0.01f || bc_screen.y < -0.01f || bc_screen.z < -0.01f) continue;
            //用重心坐标插值计算该点的深度
            float interpolated_z = bc_screen.x * pts[0].z + bc_screen.y * pts[1].z + bc_screen.z * pts[2].z;
            //用重心坐标插值计算该点的纹理坐标
            Vec2i interpolated_uv_pos = uv_pos[0] * bc_screen.x + uv_pos[1] * bc_screen.y + uv_pos[2] * bc_screen.z;
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
Vec3f WorldToScreen(Vec3f world_pos){
    return Vec3f {static_cast<float>((world_pos.x + 1.0)*kWidth/2.0),static_cast<float>((world_pos.y + 1.0)*kHeight/2.0),world_pos.z};
}

std::array<float,kPixelsCount> z_buffer{};
int main(int argc, char **argv)
{

    TGAImage image(kHeight, kWidth, TGAImage::RGB);

    //初始化z_buffer
    std::fill(z_buffer.begin(),z_buffer.end(),-std::numeric_limits<float>::max());

    std::array<Vec3f,3> screen_pos;
    std::array<Vec3f,3> world_pos;
    for(int i = 0;i<model->GetFaceSize();++i){
        auto face = model->GetVertexIndex(i);
        for(int j = 0;j<3;++j){
            auto vertex = model->GetVertByIndex(face[j]);
            screen_pos[j] = WorldToScreen(vertex);
            world_pos[j] = vertex;
        }
        Vec3f norm_vec = (world_pos[2] - world_pos[0]) ^ (world_pos[1] - world_pos[0]);
        norm_vec.normalize();
        auto intensity = norm_vec * dir;
        if(intensity > 0){
            std::array <Vec2i,3>uv_pos;
            for(int j = 0;j<3;++j) uv_pos[j] = model->GetUVByIndex(i,j);
            DrawTriangle(screen_pos,uv_pos,z_buffer,intensity,image);
        }
    }
    image.flip_vertically(); // 中心点在图片的左下角
    image.write_tga_file("../output/output.tga");

    return 0;
}