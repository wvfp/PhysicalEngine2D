#pragma once
#include "math/vector2d.hpp"
#include <stdexcept>
namespace PE2D {
	class Matrix3x3 {
	private:
		float data[3][3];

	public:
		// 默认构造函数，初始化为单位矩阵
		Matrix3x3();

		// 单值构造函数，对角元素初始化为该值，其余为 0
		explicit Matrix3x3(float value);

		// 9 个值构造函数
		Matrix3x3(float a11, float a12, float a13,
			float a21, float a22, float a23,
			float a31, float a32, float a33);

		// 数组构造函数
		explicit Matrix3x3(const float arr[9]);

		// 3 个向量（或 3 个数组）构造函数
		Matrix3x3(const float* row1, const float* row2, const float* row3);

		// 重载加法运算符
		Matrix3x3 operator+(const Matrix3x3& other) const;

		// 重载减法运算符
		Matrix3x3 operator-(const Matrix3x3& other) const;

		// 重载乘法运算符（矩阵相乘）
		Matrix3x3 operator*(const Matrix3x3& other) const;

		// 数乘
		Matrix3x3 operator*(float scalar) const;
		// 与一个2维度向量相乘
		Vector2D operator*(const Vector2D& v)const;
		// 转置
		Matrix3x3 transpose() const;
		// 缩放操作
		static Matrix3x3 scale(float sx, float sy);
		static Matrix3x3 scale(float s) { return scale(s, s); };
		// 求行列式
		float determinant() const;

		// 打印矩阵
		void print() const;

		// 旋转操作
		static Matrix3x3 rotate(float angle);

		// 平移操作
		static Matrix3x3 translate(float tx, float ty);

		// 获取元素
		float& operator()(int row, int col);
		const float& operator()(int row, int col) const;
	};
}