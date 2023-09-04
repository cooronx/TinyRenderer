
#include "tgaimage.h"
#include "window_config.h"
#include "camera.h"
#include "model.h"
#include "tinyrenderer.h"
#include "ishader.h"
#include <memory>
#include <array>
#include <Eigen/Dense>
#include <iostream>


using namespace renderer;
using Eigen::Vector4f;



Vector3f kLightDir{-4,10,-1};
Vector3f target{0,0,0};
Vector3f camera_pos{1,1,3};




std::unique_ptr<Model> model{new Model("../obj/african_head.obj")};






class Shader : public IShader{
private:
    Vector3f varing_intensity_;
    Vector3f world_z_;
    MatrixXf varing_uv_{2,3};
private:
    Matrix4f projection_;
    Matrix4f model_;
    Matrix4f view_;
    Matrix4f viewport_;

public:
    Shader(const Matrix4f &projection, const Matrix4f &model, const Matrix4f &view, const Matrix4f &viewport)
            : projection_(projection), model_(model), view_(view), viewport_(viewport) {}

private:
    Vector4f Reflect(Vector4f in,Vector4f normal){
        Vector4f reflect_vec = (in.dot(normal) * normal * 2.0f - in);
        reflect_vec.normalize();
        return reflect_vec;
    }
public:
    Vector3f GetWorldZ(){
        return world_z_;
    }
    Vector4f Vertex(int face_index, int vertex_num) override{
        //计算三角形每个顶点的光照强度
        varing_intensity_[vertex_num] = std::max(0.f,model->GetVertexNorm(face_index,vertex_num).dot(kLightDir));
        //计算三角形每个顶点的uv值
        varing_uv_.col(vertex_num) = model->GetUVByIndex(face_index,vertex_num);

        auto vertex = model->GetVertPosByIndex(face_index,vertex_num);
        auto gl_vertex = Vector4f {vertex.x(),vertex.y(),vertex.z(),1.0f};
        world_z_[vertex_num] = gl_vertex.z() - camera_pos.z();
        return viewport_ * projection_ * view_ * model_ * gl_vertex;
    }
    bool Fragment(Eigen::Vector3f barycentric, TGAColor &color) override{
        //float intensity = barycentric.dot(varing_intensity_);
        Vector2f interpolated_uv = varing_uv_ * barycentric;

        Matrix4f light_transform = view_*model_;
        Matrix4f normal_matrix = ((view_*model_).inverse()).transpose();

        //在观察空间中计算法线和光线的变换
        auto normal = model->Normal(interpolated_uv);

        auto utility_normal = Vector4f{normal.x(),normal.y(),normal.z(),1.0f};
        auto utility_light = Vector4f{kLightDir.x(),kLightDir.y(),kLightDir.z(),1.0f};
        utility_normal = normal_matrix * utility_normal;
        utility_light = light_transform * utility_light;
        //计算高光
        Vector4f out = Reflect(utility_light,utility_normal);

        float spec = pow(std::max(out.z(), 0.0f), model->Spec(interpolated_uv));
        float diffuse = std::max(0.f,utility_normal.dot(utility_light));
        TGAColor c = model->Diffuse(interpolated_uv);
        color = c;
        for (int i=0; i<3; i++) color.raw[i] = std::min<float>(5 + c.raw[i]*(diffuse + 0.4*spec), 255);
        return false;
    }
};








int main(int argc, char **argv)
{

    TGAImage image(kHeight, kWidth, TGAImage::RGB);
    std::array<Vector4f,3> world_pos;
    std::array<Vector3f,3> screen_pos;

    //初始化相机
    auto camera = std::make_unique<Camera>(camera_pos,target);
    //初始化渲染器
    auto renderer = std::make_unique<TinyRenderer>();

    //定向光的方向向量记得归一化
    kLightDir *= -1;//取反是为了计算点乘
    kLightDir.normalize();



    //这里要将视口左下脚略微上移动，不然会出现负的坐标值
    Matrix4f view_port = TinyRenderer::ViewPort(kWidth / 8, kHeight / 8, kWidth * 3 / 4, kHeight * 3 / 4);
    //透视投影矩阵
    auto projection = TinyRenderer::PerspectiveProjection(60.0f, 1.0 * kWidth / kHeight, 0.1f, 200.0f);
    //初始化look_at矩阵
    auto look_at = camera->LookAt();
    //shader实现
    auto shader = Shader(projection, Matrix4f::Identity(), look_at, view_port);

    for(int i = 0;i<model->GetFaceSize();++i){
        for(int j = 0;j<3;++j) {
            world_pos[j] = shader.Vertex(i, j);
            screen_pos[j] = TinyRenderer::HomoDivision(world_pos[j]);
        }
        renderer->DrawTriangle(screen_pos, shader.GetWorldZ(),shader, image);
    }
    image.flip_vertically(); // 中心点在图片的左下角
    image.write_tga_file("../output/output.tga");

    //画出Z_Buffer的图像
    TGAImage z_buffer_image(kWidth, kHeight, TGAImage::GRAYSCALE);
    for (int i=0; i<kWidth; i++) {
        for (int j=0; j<kHeight; j++) {
            z_buffer_image.set(i, j, TGAColor(renderer->GetZValue(i,j),1));
        }
    }

    z_buffer_image.flip_vertically();
    z_buffer_image.write_tga_file("../output/zbuffer.tga");

    return 0;
}