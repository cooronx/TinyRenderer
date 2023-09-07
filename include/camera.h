//
// Created by cooronx on 2023/8/30.
//

#ifndef TINYRENDERER_CAMERA_H
#define TINYRENDERER_CAMERA_H

#include <Eigen/Dense>
using Eigen::Matrix4f;
using Eigen::Vector3f;

namespace renderer {

class Camera {
private:
    Vector3f camera_pos_ {};
    Vector3f camera_target_ {};
    Vector3f camera_dir_ {};
    Vector3f camera_right_ {};
    Vector3f camera_up_ {};
    Vector3f world_up_ {};

public:
    explicit Camera(Vector3f, Vector3f, Vector3f world_up = { 0, 1, 0 });
    Matrix4f LookAt();
    Vector3f CameraPos();
};

} // renderer

#endif // TINYRENDERER_CAMERA_H
