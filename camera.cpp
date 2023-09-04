//
// Created by cooronx on 2023/8/30.
//

#include "camera.h"

namespace renderer {
renderer::Camera::Camera(Eigen::Vector3f camera_pos, Eigen::Vector3f camera_target, Eigen::Vector3f world_up)
    : camera_pos_(std::move(camera_pos))
    , camera_target_(std::move(camera_target))
    , world_up_(std::move(world_up))
{
}

Matrix4f Camera::LookAt()
{
    Matrix4f rotation { Matrix4f::Identity() }, translate { Matrix4f::Identity() };
    camera_dir_ = (camera_pos_ - camera_target_);
    camera_dir_.normalize();
    camera_right_ = (world_up_.cross(camera_dir_));
    camera_right_.normalize();
    camera_up_ = (camera_dir_.cross(camera_right_));
    camera_up_.normalize();

    /// Calculate
    for (int i = 0; i < 3; ++i) {
        rotation(0, i) = camera_right_[i];
        rotation(1, i) = camera_up_[i];
        rotation(2, i) = camera_dir_[i];

        translate(i, 3) = -camera_pos_[i];
    }

    return rotation * translate;
}

Vector3f Camera::CameraPos()
{
    return camera_pos_;
}
} // renderer
