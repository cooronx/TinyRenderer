#ifndef MODEL_H_
#define MODEL_H_

#include <vector>
#include "tgaimage.h"
#include <memory>
#include <Eigen/Dense>

using Eigen::Vector3f,Eigen::Vector3i,Eigen::Vector2f,Eigen::Vector2i;
class Model {
private:
	std::vector<Vector3f> verts_;
	std::vector<std::vector<Vector3i> > faces_;
    std::vector<Vector3f> norms_;
    std::vector<Vector2f> uv_;
    TGAImage diffuse_map_;
public:
	explicit Model(const char *filename);
	~Model();
	int GetFaceSize();
	int GetVertSize();
    Vector3f GetVertByIndex(int i);
	std::vector<int> GetVertexIndex(int idx);
    TGAColor Diffuse(Vector2f uv);
    Vector2f GetUVByIndex(int face_index,int vertex_cnt);
private:
    void LoadTexture(const std::string &obj_filename,const std::string &suffix,TGAImage &image);
};

#endif //MODEL_H_
