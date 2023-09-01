#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include "model.h"

Model::Model(const char *filename) : verts_(), faces_() {
    std::ifstream in;
    in.clear();
    in.open (filename, std::ifstream::in);
    if (in.fail()) {
        std::cerr<<"cant open obj file"<<std::endl;
        return ;
    }
    std::string line;
    while (!in.eof()) {
        std::getline(in, line);
        std::istringstream iss(line);
        char trash;
        if (!line.compare(0, 2, "v ")) {
            iss >> trash;
            Vector3f v;
            for(int i = 0;i<3;++i) iss >> v[i];
            verts_.push_back(v);
        }
        else if(!line.compare(0, 3, "vt ")) {
            iss >> trash >> trash;
            Vector2f uv;
            for (int i = 0; i < 2; i++) iss >> uv[i];
            uv_.push_back(uv);
        }
        else if (!line.compare(0, 3, "vn ")) {
            iss >> trash >> trash;
            Vector3f normal;
            for (int i = 0; i < 3; i++) iss >> normal[i];
            norms_.push_back(normal);
        }
        else if (!line.compare(0, 2, "f ")) {
            std::vector<Vector3i> f;
            Vector3i temp{};
            iss >> trash;
            while (iss >> temp[0] >> trash >> temp[1] >> trash >> temp[2]) {
                for(int i = 0;i<3;++i){temp[i]--;}
                f.push_back(temp);
            }
            faces_.push_back(f);
        }
    }
    std::cerr << "# v# " << verts_.size() << " f# "  << faces_.size() << std::endl;
    //加载纹理
    LoadTexture(filename,"_diffuse.tga",diffuse_map_);
}

Model::~Model() = default;

int Model::GetVertSize() {
    return (int)verts_.size();
}

int Model::GetFaceSize() {
    return (int)faces_.size();
}

std::vector<int> Model::GetVertexIndex(int idx) {
    std::vector<int> face{};
    std::vector<Vector3i> tmp = faces_[idx];
    face.reserve(tmp.size());
    for (auto & i : tmp)
        face.push_back(i[0]);
    return face;
}

Vector3f Model::GetVertByIndex(int i) {
    return verts_[i];
}

void Model::LoadTexture(const std::string &obj_filename, const std::string &suffix,TGAImage &image) {
    size_t dot_pos = obj_filename.find_last_of('.');
    if(dot_pos == std::string::npos)return ;
    auto diffuse_texture_filename = obj_filename.substr(0,dot_pos) + suffix;
    if (image.read_tga_file(diffuse_texture_filename.c_str()))
        image.flip_vertically();
    else
        std::cerr << "diffuse texture file read failed" << std::endl;

}

TGAColor Model::Diffuse(Vector2f uv) {
    return diffuse_map_.get(static_cast <int>(uv.x()), static_cast <int> (uv.y()));
}

Vector2f Model::GetUVByIndex(int face_index, int vertex_cnt) {
    int index = faces_[face_index][vertex_cnt][1];
    return {static_cast<int>(uv_[index].x() * diffuse_map_.get_width()),
            static_cast<int>(uv_[index].y() * diffuse_map_.get_height())};
}

