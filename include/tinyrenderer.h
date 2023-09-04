//
// Created by cooronx on 2023/8/31.
//

#ifndef TINYRENDERER_TINYRENDERER_H
#define TINYRENDERER_TINYRENDERER_H
#include "window_config.h"
#include "ishader.h"
#include <Eigen/Dense>

using namespace Eigen;


namespace renderer{
    class TinyRenderer {
        private:
            std::array < std::array<float,kWidth>,kHeight > z_buffer{};
        public:
            TinyRenderer();
            void DrawTriangle(const std::array<Vector3f, 3>& pts,const Vector3f &world_z,IShader &shader,TGAImage &image);
            float GetZValue(int i,int j);
        public:
            static Matrix4f PerspectiveProjection(float eye_fov, float aspect_ratio,float zNear, float zFar);
            static Matrix4f ViewPort(int x, int y, int w, int h);
            static Vector3f HomoDivision(Vector4f vec);
        private:
            static Vector3f barycentric(const std::array<Vector3f,3> &pts,const Vector2i& P);
    };
}//renderer



 
#endif //TINYRENDERER_TINYRENDERER_H
