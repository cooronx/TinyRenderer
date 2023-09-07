//
// Created by cooronx on 2023/8/30.
//

#ifndef TINYRENDERER_ISHADER_H
#define TINYRENDERER_ISHADER_H

#include "tgaimage.h"
#include <Eigen/Dense>


using Eigen::Vector3f;
using Eigen::Vector4f;

class IShader {
public:
    IShader() = default;

    virtual Vector4f Vertex(int face_index, int vertex_num) = 0;

    virtual bool Fragment(Vector3f barycentric, TGAColor& color) = 0;
};

#endif // TINYRENDERER_ISHADER_H
