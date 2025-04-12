#include "math/matrix3x3.hpp"
#include <cmath>
#include <iostream>
namespace PE2D
{
	// 默认构造函数，初始化为单位矩阵
	Matrix3x3::Matrix3x3() {
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				data[i][j] = (i == j) ? 1.0f : 0.0f;
			}
		}
	}

	// 单值构造函数，对角元素初始化为该值，其余为 0
	Matrix3x3::Matrix3x3(float value) {
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				data[i][j] = (i == j) ? value : 0.0f;
			}
		}
	}

	// 9 个值构造函数
	Matrix3x3::Matrix3x3(float a11, float a12, float a13,
		float a21, float a22, float a23,
		float a31, float a32, float a33) {
		data[0][0] = a11;
		data[0][1] = a12;
		data[0][2] = a13;
		data[1][0] = a21;
		data[1][1] = a22;
		data[1][2] = a23;
		data[2][0] = a31;
		data[2][1] = a32;
		data[2][2] = a33;
	}

	// 数组构造函数
	Matrix3x3::Matrix3x3(const float arr[9]) {
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				data[i][j] = arr[i * 3 + j];
			}
		}
	}

	// 3 个向量（或 3 个数组）构造函数
	Matrix3x3::Matrix3x3(const float* row1, const float* row2, const float* row3) {
		for (int j = 0; j < 3; ++j) {
			data[0][j] = row1[j];
			data[1][j] = row2[j];
			data[2][j] = row3[j];
		}
	}

	// 重载加法运算符
	Matrix3x3 Matrix3x3::operator+(const Matrix3x3& other) const {
		return Matrix3x3(
			data[0][0] + other.data[0][0], data[0][1] + other.data[0][1], data[0][2] + other.data[0][2],
			data[1][0] + other.data[1][0], data[1][1] + other.data[1][1], data[1][2] + other.data[1][2],
			data[2][0] + other.data[2][0], data[2][1] + other.data[2][1], data[2][2] + other.data[2][2]
		);
	}

	// 重载减法运算符
	Matrix3x3 Matrix3x3::operator-(const Matrix3x3& other) const {
		return Matrix3x3(
			data[0][0] - other.data[0][0], data[0][1] - other.data[0][1], data[0][2] - other.data[0][2],
			data[1][0] - other.data[1][0], data[1][1] - other.data[1][1], data[1][2] - other.data[1][2],
			data[2][0] - other.data[2][0], data[2][1] - other.data[2][1], data[2][2] - other.data[2][2]
		);
	}

	// 重载乘法运算符（矩阵相乘）
	Matrix3x3 Matrix3x3::operator*(const Matrix3x3& other) const {
		Matrix3x3 result;
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				result.data[i][j] = 0;
				for (int k = 0; k < 3; ++k) {
					result.data[i][j] += data[i][k] * other.data[k][j];
				}
			}
		}
		return result;
	}

	// 数乘
	Matrix3x3 Matrix3x3::operator*(float scalar) const {
		return Matrix3x3(
			data[0][0] * scalar, data[0][1] * scalar, data[0][2] * scalar,
			data[1][0] * scalar, data[1][1] * scalar, data[1][2] * scalar,
			data[2][0] * scalar, data[2][1] * scalar, data[2][2] * scalar
		);
	}

	Vector2D Matrix3x3::operator*(const Vector2D& v)const {
		return Vector2D(data[0][0]*v.x()+data[0][1]*v.y()+data[0][2],
			data[1][0]*v.x()+data[1][1]*v.y()+data[1][2]);
	}
	// 转置
	Matrix3x3 Matrix3x3::transpose() const {
		return Matrix3x3(
			data[0][0], data[1][0], data[2][0],
			data[0][1], data[1][1], data[2][1],
			data[0][2], data[1][2], data[2][2]
		);
	}
	// 缩放操作

	Matrix3x3 Matrix3x3::scale(float sx, float sy) {
		return Matrix3x3(
			sx, 0, 0,
			0, sy, 0,
			0, 0, 1
		);
	}
	// 求行列式
	float Matrix3x3::determinant() const {
		return data[0][0] * (data[1][1] * data[2][2] - data[1][2] * data[2][1]) -
			data[0][1] * (data[1][0] * data[2][2] - data[1][2] * data[2][0]) +
			data[0][2] * (data[1][0] * data[2][1] - data[1][1] * data[2][0]);
	}

	// 打印矩阵
	void Matrix3x3::print() const {
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				std::cout << data[i][j] << " ";
			}
			std::cout << std::endl;
		}
	}

	// 旋转操作
	Matrix3x3 Matrix3x3::rotate(float angle) {
		float cosAngle = std::cos(angle);
		float sinAngle = std::sin(angle);
		return Matrix3x3(
			cosAngle, -sinAngle, 0,
			sinAngle, cosAngle, 0,
			0, 0, 1
		);
	}

	// 平移操作
	Matrix3x3 Matrix3x3::translate(float tx, float ty) {
		return Matrix3x3(
			1, 0, tx,
			0, 1, ty,
			0, 0, 1
		);
	}

	// 获取元素
	float& Matrix3x3::operator()(int row, int col) {
		return data[row][col];
	}

	const float& Matrix3x3::operator()(int row, int col) const {
		return data[row][col];
	}
}