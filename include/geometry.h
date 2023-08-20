#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include <cmath>
#include <ostream>
#include <vector>
<<<<<<< HEAD

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
=======
>>>>>>> origin/main

template <class t> struct Vec2 {
    t x, y;
    Vec2<t>() : x(t()), y(t()) {}
    Vec2<t>(t _x, t _y) : x(_x), y(_y) {}
<<<<<<< HEAD
    Vec2<t>(const Vec2<t> &v) : x(t()), y(t()) { *this = v; }
    Vec2<t> & operator =(const Vec2<t> &v) {
=======
    Vec2<t>(const Vec2<t>& v) : x(t()), y(t()) { *this = v; }
    Vec2<t>& operator =(const Vec2<t>& v) {
>>>>>>> origin/main
        if (this != &v) {
            x = v.x;
            y = v.y;
        }
        return *this;
    }
<<<<<<< HEAD
    Vec2<t> operator +(const Vec2<t> &V) const { return Vec2<t>(x+V.x, y+V.y); }
    Vec2<t> operator -(const Vec2<t> &V) const { return Vec2<t>(x-V.x, y-V.y); }
    Vec2<t> operator *(float f)          const { return Vec2<t>(x*f, y*f); }
    t& operator[](const int i) { if (x<=0) return x; else return y; }
=======
    Vec2<t> operator +(const Vec2<t>& V) const { return Vec2<t>(x + V.x, y + V.y); }
    Vec2<t> operator -(const Vec2<t>& V) const { return Vec2<t>(x - V.x, y - V.y); }
    Vec2<t> operator *(float f)          const { return Vec2<t>(x * f, y * f); }
    t& operator[](const int i) { if (x <= 0) return x; else return y; }
>>>>>>> origin/main
    template <class > friend std::ostream& operator<<(std::ostream& s, Vec2<t>& v);
};

template <class t> struct Vec3 {
    t x, y, z;
    Vec3<t>() : x(t()), y(t()), z(t()) { }
    Vec3<t>(t _x, t _y, t _z) : x(_x), y(_y), z(_z) {}
<<<<<<< HEAD
    template <class u> explicit Vec3<t>(const Vec3<u> &v);
    Vec3<t>(const Vec3<t> &v) : x(t()), y(t()), z(t()) { *this = v; }
    Vec3<t> & operator =(const Vec3<t> &v) {
=======
    template <class u> Vec3<t>(const Vec3<u>& v);
    Vec3<t>(const Vec3<t>& v) : x(t()), y(t()), z(t()) { *this = v; }
    Vec3<t>& operator =(const Vec3<t>& v) {
>>>>>>> origin/main
        if (this != &v) {
            x = v.x;
            y = v.y;
            z = v.z;
        }
        return *this;
    }
<<<<<<< HEAD
    Vec3<t> operator ^(const Vec3<t> &v) const { return Vec3<t>(y*v.z-z*v.y, z*v.x-x*v.z, x*v.y-y*v.x); }
    Vec3<t> operator +(const Vec3<t> &v) const { return Vec3<t>(x+v.x, y+v.y, z+v.z); }
    Vec3<t> operator -(const Vec3<t> &v) const { return Vec3<t>(x-v.x, y-v.y, z-v.z); }
    Vec3<t> operator *(float f)          const { return Vec3<t>(x*f, y*f, z*f); }
    t       operator *(const Vec3<t> &v) const { return x*v.x + y*v.y + z*v.z; }
    [[nodiscard]] float norm () const { return std::sqrt(x*x+y*y+z*z); }
    Vec3<t> & Normalize(t l=1) { *this = (*this)*(l/norm()); return *this; }
    t& operator[](const int i) { if (i<=0) return x; else if (i==1) return y; else return z; }
=======
    Vec3<t> operator ^(const Vec3<t>& v) const { return Vec3<t>(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x); }
    Vec3<t> operator +(const Vec3<t>& v) const { return Vec3<t>(x + v.x, y + v.y, z + v.z); }
    Vec3<t> operator -(const Vec3<t>& v) const { return Vec3<t>(x - v.x, y - v.y, z - v.z); }
    Vec3<t> operator *(float f)          const { return Vec3<t>(x * f, y * f, z * f); }
    t       operator *(const Vec3<t>& v) const { return x * v.x + y * v.y + z * v.z; }
    float norm() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3<t>& normalize(t l = 1) { *this = (*this) * (l / norm()); return *this; }
    t& operator[](const int i) { if (i <= 0) return x; else if (i == 1) return y; else return z; }
>>>>>>> origin/main
    template <class > friend std::ostream& operator<<(std::ostream& s, Vec3<t>& v);
};

typedef Vec2<float> Vec2f;
typedef Vec2<int>   Vec2i;
typedef Vec3<float> Vec3f;
typedef Vec3<int>   Vec3i;

<<<<<<< HEAD
template <> template <> Vec3<int>::Vec3(const Vec3<float> &v);
template <> template <> Vec3<float>::Vec3(const Vec3<int> &v);
=======
template <> template <> Vec3<int>::Vec3(const Vec3<float>& v);
template <> template <> Vec3<float>::Vec3(const Vec3<int>& v);
>>>>>>> origin/main


template <class t> std::ostream& operator<<(std::ostream& s, Vec2<t>& v) {
    s << "(" << v.x << ", " << v.y << ")\n";
    return s;
}

template <class t> std::ostream& operator<<(std::ostream& s, Vec3<t>& v) {
    s << "(" << v.x << ", " << v.y << ", " << v.z << ")\n";
    return s;
}

<<<<<<< HEAD


const int DEFAULT_ALLOC=4;

class Matrix {
    std::vector< std::vector<float> > m;
    int rows, cols;
public:
    Matrix(int r=DEFAULT_ALLOC, int c=DEFAULT_ALLOC);
    inline int nrows();
    inline int ncols();

    static Matrix identity(int dimensions);
    std::vector<float>& operator[](const int i);
    Matrix operator*(const Matrix& a);
    Matrix transpose();
    Matrix inverse();
=======
//////////////////////////////////////////////////////////////////////////////////////////////

const int DEFAULT_ALLOC = 4;

class Matrix {
    std::vector< std::vector<float> > m;
    int rows{}, cols{};
public:
    explicit Matrix(int r = DEFAULT_ALLOC, int c = DEFAULT_ALLOC);
    inline int RowSize() const;
    inline int ColumnSize() const;

    static Matrix Identity(int dimensions);
    std::vector<float>& operator[](int i);
    Matrix operator*(const Matrix& a);
    Matrix Transpose();
    Matrix Inverse();
>>>>>>> origin/main

    friend std::ostream& operator<<(std::ostream& s, Matrix& m);
};

<<<<<<< HEAD
#endif //__GEOMETRY_H__
=======
/////////////////////////////////////////////////////////////////////////////////////////////


#endif //__GEOMETRY_H__
>>>>>>> origin/main
