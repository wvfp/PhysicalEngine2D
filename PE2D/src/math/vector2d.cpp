#include "math/vector2d.hpp"
#include "math/math_utils.hpp"
#include <exception>
#include <string>
#include <cmath>
#include <stdexcept>

namespace PE2D {
	// 构造函数的实现
	Vector2D::Vector2D(float x, float y) : m_x(x), m_y(y) {}

	// 加法重载的实现
	Vector2D Vector2D::operator+(const Vector2D& other) const {
		return Vector2D(m_x + other.m_x, m_y + other.m_y);
	}

	// 减法重载的实现（差值向量）
	Vector2D Vector2D::operator-(const Vector2D& other) const {
		return Vector2D(m_x - other.m_x, m_y - other.m_y);
	}
	// 向量除以向量的方法
	Vector2D Vector2D::operator/(const Vector2D& other) const {
		Vector2D result;
		if (abs(other.m_x) < 1e-10) {
			result.m_x = 0.0f;
		}
		else {
			result.m_x = m_x / other.m_x;
		}
		if (abs(other.m_y) < 1e-10) {
			result.m_y = 0;
		}
		else {
			result.m_y = m_y / other.m_y;
		}
		return result;
	}
	// 标量乘法重载的实现（向量缩放）
	Vector2D Vector2D::operator*(float scalar) const {
		return Vector2D(m_x * scalar, m_y * scalar);
	}
	Vector2D Vector2D::operator*(const Vector2D& other) const {
		return Vector2D(m_x * other.m_x, m_y * other.m_y);
	}
	// 标量除法重载的实现
	Vector2D Vector2D::operator/(float scalar) const {
		if (scalar == 0.0f) {
			throw std::runtime_error("Division by zero in Vector2D::operator/");
		}
		return Vector2D(m_x / scalar, m_y / scalar);
	}

	// 点乘（内积）的实现
	float Vector2D::dot(const Vector2D& other) const {
		return m_x * other.m_x + m_y * other.m_y;
	}

	// 二维向量的叉乘（结果为标量，计算有向面积）的实现
	float Vector2D::cross(const Vector2D& other) const {
		return m_x * other.m_y - m_y * other.m_x;
	}

	// 二维向量的叉乘扩展到三维空间（这里简单返回一个z分量为叉乘结果的三维向量形式示意，实际可按需完善）
	Vector2D::Virtual3DVector Vector2D::extendedCross(const Vector2D& other) const {
		return { 0.0f, 0.0f, cross(other) };
	}

	// 下标访问重载的实现
	const float& Vector2D::operator[](const int index) const {
		if (index == 0) {
			return m_x;
		}
		else if (index == 1) {
			return m_y;
		}
		else {
			throw std::out_of_range("Index out of range in Vector2D::operator[]");
		}
	}

	// 模拟类似map的访问方式的实现
	float Vector2D::operator[](const std::string& key) const {
		static const std::string keyX = "x";
		static const std::string keyY = "y";
		if (key == keyX) {
			return m_x;
		}
		else if (key == keyY) {
			return m_y;
		}
		else {
			throw std::invalid_argument("Invalid key in Vector2D::operator[]");
		}
	}

	// 取模运算（计算向量的模长）的实现
	float Vector2D::magnitude() const {
		return std::sqrt(m_x * m_x + m_y * m_y);
	}
	float Vector2D::magnitudeSq()const {
		return m_x * m_x + m_y * m_y;
	}

	// 单位化（归一化）的实现
	Vector2D Vector2D::normalize() const {
		float mag = magnitude();
		if (mag == 0.0f) {
			return Vector2D(0.0f, 0.0f);
		}
		return Vector2D(m_x / mag, m_y / mag);
	}

	// 计算两个向量的夹角（弧度制）的实现
	float Vector2D::angle(const Vector2D& other) const {
		float dotProduct = dot(other);
		float magSelf = magnitude();
		float magOther = other.magnitude();
		if (magSelf == 0.0f || magOther == 0.0f) {
			return 0.0f;
		}
		return std::acos(dotProduct / (magSelf * magOther));
	}
	// 计算当前向量在另一个向量上的投影的实现
	Vector2D Vector2D::projection(const Vector2D& other) const {
		float dotProduct = dot(other);
		float magOtherSquared = other.magnitude();
		magOtherSquared *= magOtherSquared;
		if (magOtherSquared == 0.0f) {
			return Vector2D(0.0f, 0.0f);
		}
		float scalar = dotProduct / magOtherSquared;
		return other * scalar;
	}

	// 旋转向量（绕原点按弧度制旋转）的实现
	Vector2D Vector2D::rotate(float angle) const {
		float cosA = std::cos(angle);
		float sinA = std::sin(angle);
		return Vector2D(m_x * cosA - m_y * sinA, m_x * sinA + m_y * cosA);
	}

	// 平移向量的实现
	void Vector2D::translate(float dx, float dy) {
		m_x += dx;
		m_y += dy;
	}

	// 计算一个向量关于另一个向量的反射向量的实现
	Vector2D Vector2D::reflect(const Vector2D& normal) const {
		Vector2D n = normal.normalize();
		float dotProd = dot(n) * 2;
		return *this - n * dotProd;
	}

	// 计算两个向量之间的插值向量（线性插值）的实现
	Vector2D Vector2D::interpolate(const Vector2D& other, float t) const {
		return Vector2D(m_x + t * (other.m_x - m_x), m_y + t * (other.m_y - m_y));
	}

	// 将向量转换为极坐标形式（返回极径和极角）的实现
	Vector2D::PolarCoordinates Vector2D::toPolar() const {
		PolarCoordinates polar;
		polar.radius = magnitude();
		polar.angle = std::atan2(m_y, m_x);
		return polar;
	}

	// 向量正交化（简单示意，这里使用二维特殊情况，实际可使用更通用的Gram-Schmidt过程）的实现
	Vector2D Vector2D::orthogonalize() const {
		if (m_x == 0 && m_y == 0) {
			return Vector2D(0.0f, 0.0f);
		}
		return Vector2D(-m_y, m_x).normalize();
	}

	// 向量的线性组合的实现
	Vector2D Vector2D::linearCombination(float scalar1, const Vector2D& v1, float scalar2, const Vector2D& v2) const {
		return v1 * scalar1 + v2 * scalar2;
	}

	// 重载标量乘法运算符（将标量放在左边的情况）的实现
	Vector2D operator*(float scalar, const Vector2D& vector) {
		return vector * scalar;
	}
}