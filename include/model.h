#ifndef MODEL_H_
#define MODEL_H_

#include "tgaimage.h"
#include <Eigen/Dense>
#include <vector>

using Eigen::Vector3f, Eigen::Vector3i, Eigen::Vector2f, Eigen::Vector2i;
class Model {
private:
    std::vector<Vector3f> verts_;
    std::vector<std::vector<Vector3i>> faces_;
    std::vector<Vector3f> norms_;
    std::vector<Vector2f> uv_;
    TGAImage diffuse_map_;
    TGAImage normal_map_;
    TGAImage spec_map_;

public:
    explicit Model(const char* filename);
    ~Model();
    int GetFaceSize();
    int GetVertSize();
    Vector3f GetVertByIndex(int i);
    std::vector<int> GetVertexIndex(int idx);
    TGAColor Diffuse(Vector2f uvf);
    Vector3f Normal(Vector2f uvf);
    float Spec(Vector2f uvf);
    Vector2f GetUVByIndex(int face_index, int vertex_cnt);
    Vector3f GetVertexNorm(size_t face_index, size_t vertex_num);
    Vector3f GetVertPosByIndex(size_t face_index, size_t vertex_num);

private:
    static void LoadTexture(const std::string& obj_filename, const std::string& suffix, TGAImage& image);
};

#endif // MODEL_H_
