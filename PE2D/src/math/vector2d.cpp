#include "math/vector2d.hpp"
#include "math/math_utils.hpp"
#include <exception>
#include <string>
#include <cmath>
#include <stdexcept>

namespace PE2D {
	// ���캯����ʵ��
	Vector2D::Vector2D(float x, float y) : m_x(x), m_y(y) {}

	// �ӷ����ص�ʵ��
	Vector2D Vector2D::operator+(const Vector2D& other) const {
		return Vector2D(m_x + other.m_x, m_y + other.m_y);
	}

	// �������ص�ʵ�֣���ֵ������
	Vector2D Vector2D::operator-(const Vector2D& other) const {
		return Vector2D(m_x - other.m_x, m_y - other.m_y);
	}
	// �������������ķ���
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
	// �����˷����ص�ʵ�֣��������ţ�
	Vector2D Vector2D::operator*(float scalar) const {
		return Vector2D(m_x * scalar, m_y * scalar);
	}
	Vector2D Vector2D::operator*(const Vector2D& other) const {
		return Vector2D(m_x * other.m_x, m_y * other.m_y);
	}
	// �����������ص�ʵ��
	Vector2D Vector2D::operator/(float scalar) const {
		if (scalar == 0.0f) {
			throw std::runtime_error("Division by zero in Vector2D::operator/");
		}
		return Vector2D(m_x / scalar, m_y / scalar);
	}

	// ��ˣ��ڻ�����ʵ��
	float Vector2D::dot(const Vector2D& other) const {
		return m_x * other.m_x + m_y * other.m_y;
	}

	// ��ά�����Ĳ�ˣ����Ϊ���������������������ʵ��
	float Vector2D::cross(const Vector2D& other) const {
		return m_x * other.m_y - m_y * other.m_x;
	}

	// ��ά�����Ĳ����չ����ά�ռ䣨����򵥷���һ��z����Ϊ��˽������ά������ʽʾ�⣬ʵ�ʿɰ������ƣ�
	Vector2D::Virtual3DVector Vector2D::extendedCross(const Vector2D& other) const {
		return { 0.0f, 0.0f, cross(other) };
	}

	// �±�������ص�ʵ��
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

	// ģ������map�ķ��ʷ�ʽ��ʵ��
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

	// ȡģ���㣨����������ģ������ʵ��
	float Vector2D::magnitude() const {
		return std::sqrt(m_x * m_x + m_y * m_y);
	}
	float Vector2D::magnitudeSq()const {
		return m_x * m_x + m_y * m_y;
	}

	// ��λ������һ������ʵ��
	Vector2D Vector2D::normalize() const {
		float mag = magnitude();
		if (mag == 0.0f) {
			return Vector2D(0.0f, 0.0f);
		}
		return Vector2D(m_x / mag, m_y / mag);
	}

	// �������������ļнǣ������ƣ���ʵ��
	float Vector2D::angle(const Vector2D& other) const {
		float dotProduct = dot(other);
		float magSelf = magnitude();
		float magOther = other.magnitude();
		if (magSelf == 0.0f || magOther == 0.0f) {
			return 0.0f;
		}
		return std::acos(dotProduct / (magSelf * magOther));
	}
	// ���㵱ǰ��������һ�������ϵ�ͶӰ��ʵ��
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

	// ��ת��������ԭ�㰴��������ת����ʵ��
	Vector2D Vector2D::rotate(float angle) const {
		float cosA = std::cos(angle);
		float sinA = std::sin(angle);
		return Vector2D(m_x * cosA - m_y * sinA, m_x * sinA + m_y * cosA);
	}

	// ƽ��������ʵ��
	void Vector2D::translate(float dx, float dy) {
		m_x += dx;
		m_y += dy;
	}

	// ����һ������������һ�������ķ���������ʵ��
	Vector2D Vector2D::reflect(const Vector2D& normal) const {
		Vector2D n = normal.normalize();
		float dotProd = dot(n) * 2;
		return *this - n * dotProd;
	}

	// ������������֮��Ĳ�ֵ���������Բ�ֵ����ʵ��
	Vector2D Vector2D::interpolate(const Vector2D& other, float t) const {
		return Vector2D(m_x + t * (other.m_x - m_x), m_y + t * (other.m_y - m_y));
	}

	// ������ת��Ϊ��������ʽ�����ؼ����ͼ��ǣ���ʵ��
	Vector2D::PolarCoordinates Vector2D::toPolar() const {
		PolarCoordinates polar;
		polar.radius = magnitude();
		polar.angle = std::atan2(m_y, m_x);
		return polar;
	}

	// ��������������ʾ�⣬����ʹ�ö�ά���������ʵ�ʿ�ʹ�ø�ͨ�õ�Gram-Schmidt���̣���ʵ��
	Vector2D Vector2D::orthogonalize() const {
		if (m_x == 0 && m_y == 0) {
			return Vector2D(0.0f, 0.0f);
		}
		return Vector2D(-m_y, m_x).normalize();
	}

	// ������������ϵ�ʵ��
	Vector2D Vector2D::linearCombination(float scalar1, const Vector2D& v1, float scalar2, const Vector2D& v2) const {
		return v1 * scalar1 + v2 * scalar2;
	}

	// ���ر����˷��������������������ߵ��������ʵ��
	Vector2D operator*(float scalar, const Vector2D& vector) {
		return vector * scalar;
	}
}