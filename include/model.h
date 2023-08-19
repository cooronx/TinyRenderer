#ifndef MODEL_H_
#define MODEL_H_

#include <vector>
#include "geometry.h"
#include "tgaimage.h"
#include <memory>

class Model {
private:
	std::vector<Vec3f> verts_;
	std::vector<std::vector<Vec3i> > faces_;
    std::vector<Vec3f> norms_;
    std::vector<Vec2f> uv_;
    TGAImage diffuse_map_;
public:
	explicit Model(const char *filename);
	~Model();
	int GetFaceSize();
	int GetVertSize();
	Vec3f GetVertByIndex(int i);
	std::vector<int> GetVertexIndex(int idx);
    TGAColor Diffuse(Vec2i uv);
    Vec2i GetUVByIndex(int face_index,int vertex_cnt);
private:
    void LoadTexture(const std::string &obj_filename,const std::string &suffix,TGAImage &image);
};

#endif //MODEL_H_
