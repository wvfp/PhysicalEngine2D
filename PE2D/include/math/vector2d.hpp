#pragma once

#include <string>
#include <cmath>
namespace PE2D {
	class Vector2D {
	private:
		float m_x;
		float m_y;

	public:
		// 构造函数
		Vector2D(float x = 0.0f, float y = 0.0f);
		// 获取 x 和 y 分量
		float x() const { return m_x; }
		float y() const { return m_y; }
		// 设置 x 和 y 分量
		void setX(float x) { m_x = x; }
		void setY(float y) { m_y = y; }
		//-Vector2D 的重载
		Vector2D operator-() const { return Vector2D(-m_x, -m_y); }
		// 加法重载
		Vector2D operator+(const Vector2D& other) const;
		Vector2D operator+=(const Vector2D& other) { *this = *this + other; return *this; }
		// 减法重载（差值向量）
		Vector2D operator-(const Vector2D& other) const;
		Vector2D operator-=(const Vector2D& other) { *this = *this - other; return *this; }

		// 标量乘法重载（向量缩放）
		Vector2D operator*(float scalar) const;
		Vector2D operator*=(float scalar) { *this = *this * scalar; return *this; }
		Vector2D operator*(const Vector2D& other) const;
		Vector2D operator*=(const Vector2D& other) { *this = *this * other; return *this; }

		// 标量除法重载
		Vector2D operator/(float scalar) const;
		Vector2D operator/=(float scalar) { *this = *this / scalar; return *this; }
		Vector2D operator/(const Vector2D& other) const;
		Vector2D operator/=(const Vector2D& other) { *this = *this / other; return *this; }

		bool operator==(const Vector2D& other) const {
			return (m_x == other.m_x && m_y == other.m_y);
		}

		// 点乘（内积）
		float dot(const Vector2D& other) const;

		// 二维向量的叉乘（结果为标量，计算有向面积）
		float cross(const Vector2D& other) const;
		// 二维向量的叉乘扩展到三维空间（这里简单返回一个z分量为叉乘结果的三维向量形式示意)
		struct Virtual3DVector {
			float x;
			float y;
			float z;
		};
		Virtual3DVector extendedCross(const Vector2D& other) const;

		// 下标访问重载
		const float& operator[](const int index) const;
		// 模拟类似map的访问方式
		float operator[](const std::string& key) const;

		// 取模运算（计算向量的模长）
		float magnitude() const;
		float magnitudeSq()const;

		// 单位化（归一化）
		Vector2D normalize() const;

		// 计算两个向量的夹角（弧度制）
		float angle(const Vector2D& other) const;

		// 计算当前向量在另一个向量上的投影
		Vector2D projection(const Vector2D& other) const;

		// 旋转向量（绕原点按弧度制旋转）
		Vector2D rotate(float angle) const;

		// 平移向量
		void translate(float dx, float dy);

		// 计算一个向量关于另一个向量的反射向量
		Vector2D reflect(const Vector2D& normal) const;

		// 计算两个向量之间的插值向量（线性插值）
		Vector2D interpolate(const Vector2D& other, float t) const;

		// 将向量转换为极坐标形式（返回极径和极角）
		struct PolarCoordinates {
			float radius;
			float angle;
		};
		PolarCoordinates toPolar() const;

		// 向量正交化（简单示意，这里使用二维特殊情况，实际可使用更通用的Gram-Schmidt过程）
		Vector2D orthogonalize() const;

		// 向量的线性组合
		Vector2D linearCombination(float scalar1, const Vector2D& v1, float scalar2, const Vector2D& v2) const;
	};

	// 重载标量乘法运算符（将标量放在左边的情况）
	Vector2D operator*(float scalar, const Vector2D& vector);
}