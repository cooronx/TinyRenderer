#include <vector>
#include <cassert>
#include <cmath>
#include <iostream>
#include "geometry.h"

template <> template <> Vec3<int>::Vec3(const Vec3<float>& v) : x(lround(v.x)), y(lround(v.y)), z(lround(v.z)) {}
template <> template <> Vec3<float>::Vec3(const Vec3<int>& v) : x(v.x), y(v.y), z(v.z) {}


Matrix::Matrix(int r, int c) : m_(std::vector<std::vector<float> >(r, std::vector<float>(c, 0.f))), rows_(r), cols_(c) { }

int Matrix::RowSize() const {
    return rows_;
}

int Matrix::ColumnSize() const {
    return cols_;
}

Matrix Matrix::Identity(int dimensions) {
    Matrix E(dimensions, dimensions);
    for (int i=0; i<dimensions; i++) {
        for (int j=0; j<dimensions; j++) {
            E[i][j] = (i==j ? 1.f : 0.f);
        }
    }
    return E;
}

std::vector<float>& Matrix::operator[](const int i) {
    assert(i>=0 && i < rows_);
    return m_[i];
}

Matrix Matrix::operator*(const Matrix& a) {
    assert(cols_ == a.rows_);
    Matrix result(rows_, a.cols_);
    for (int i=0; i < rows_; i++) {
        for (int j=0; j<a.cols_; j++) {
            result.m_[i][j] = 0.f;
            for (int k=0; k < cols_; k++) {
                result.m_[i][j] += m_[i][k] * a.m_[k][j];
            }
        }
    }
    return result;
}

Matrix Matrix::Transpose() {
    Matrix result(cols_, rows_);
    for(int i=0; i < rows_; i++)
        for(int j=0; j < cols_; j++)
            result[j][i] = m_[i][j];
    return result;
}

Matrix Matrix::Inverse() {
    assert(rows_ == cols_);
    // augmenting the square matrix with the Identity matrix of the same dimensions a => [ai]
    Matrix result(rows_, cols_ * 2);
    for(int i=0; i < rows_; i++)
        for(int j=0; j < cols_; j++)
            result[i][j] = m_[i][j];
    for(int i=0; i < rows_; i++)
        result[i][i + cols_] = 1;
    // first pass
    for (int i=0; i < rows_ - 1; i++) {
        // normalize the first row
        for(int j= result.cols_ - 1; j >= 0; j--)
            result[i][j] /= result[i][i];
        for (int k=i+1; k < rows_; k++) {
            float coeff = result[k][i];
            for (int j=0; j<result.cols_; j++) {
                result[k][j] -= result[i][j]*coeff;
            }
        }
    }
    // normalize the last row
    for(int j= result.cols_ - 1; j >= rows_ - 1; j--)
        result[rows_ - 1][j] /= result[rows_ - 1][rows_ - 1];
    // second pass
    for (int i= rows_ - 1; i > 0; i--) {
        for (int k=i-1; k>=0; k--) {
            float coeff = result[k][i];
            for (int j=0; j<result.cols_; j++) {
                result[k][j] -= result[i][j]*coeff;
            }
        }
    }
    // cut the Identity matrix back
    Matrix truncate(rows_, cols_);
    for(int i=0; i < rows_; i++)
        for(int j=0; j < cols_; j++)
            truncate[i][j] = result[i][j + cols_];
    return truncate;
}

std::ostream& operator<<(std::ostream& s, Matrix& m) {
    for (int i=0; i< m.RowSize(); i++)  {
        for (int j=0; j< m.ColumnSize(); j++) {
            s << m[i][j];
            if (j< m.ColumnSize() - 1) s << "\t";
        }
        s << "\n";
    }
    return s;
}

Matrix::Matrix(Matrix &&that) noexcept {
    this->rows_ = that.rows_;
    this->cols_ = that.cols_;
    this->m_ = that.m_;
}
