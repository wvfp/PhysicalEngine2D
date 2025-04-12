#pragma once

#include <string>
#include <cmath>
namespace PE2D {
	class Vector2D {
	private:
		float m_x;
		float m_y;

	public:
		// ���캯��
		Vector2D(float x = 0.0f, float y = 0.0f);
		// ��ȡ x �� y ����
		float x() const { return m_x; }
		float y() const { return m_y; }
		// ���� x �� y ����
		void setX(float x) { m_x = x; }
		void setY(float y) { m_y = y; }
		//-Vector2D ������
		Vector2D operator-() const { return Vector2D(-m_x, -m_y); }
		// �ӷ�����
		Vector2D operator+(const Vector2D& other) const;
		Vector2D operator+=(const Vector2D& other) { *this = *this + other; return *this; }
		// �������أ���ֵ������
		Vector2D operator-(const Vector2D& other) const;
		Vector2D operator-=(const Vector2D& other) { *this = *this - other; return *this; }

		// �����˷����أ��������ţ�
		Vector2D operator*(float scalar) const;
		Vector2D operator*=(float scalar) { *this = *this * scalar; return *this; }
		Vector2D operator*(const Vector2D& other) const;
		Vector2D operator*=(const Vector2D& other) { *this = *this * other; return *this; }

		// ������������
		Vector2D operator/(float scalar) const;
		Vector2D operator/=(float scalar) { *this = *this / scalar; return *this; }
		Vector2D operator/(const Vector2D& other) const;
		Vector2D operator/=(const Vector2D& other) { *this = *this / other; return *this; }

		bool operator==(const Vector2D& other) const {
			return (m_x == other.m_x && m_y == other.m_y);
		}

		// ��ˣ��ڻ���
		float dot(const Vector2D& other) const;

		// ��ά�����Ĳ�ˣ����Ϊ�������������������
		float cross(const Vector2D& other) const;
		// ��ά�����Ĳ����չ����ά�ռ䣨����򵥷���һ��z����Ϊ��˽������ά������ʽʾ��)
		struct Virtual3DVector {
			float x;
			float y;
			float z;
		};
		Virtual3DVector extendedCross(const Vector2D& other) const;

		// �±��������
		const float& operator[](const int index) const;
		// ģ������map�ķ��ʷ�ʽ
		float operator[](const std::string& key) const;

		// ȡģ���㣨����������ģ����
		float magnitude() const;
		float magnitudeSq()const;

		// ��λ������һ����
		Vector2D normalize() const;

		// �������������ļнǣ������ƣ�
		float angle(const Vector2D& other) const;

		// ���㵱ǰ��������һ�������ϵ�ͶӰ
		Vector2D projection(const Vector2D& other) const;

		// ��ת��������ԭ�㰴��������ת��
		Vector2D rotate(float angle) const;

		// ƽ������
		void translate(float dx, float dy);

		// ����һ������������һ�������ķ�������
		Vector2D reflect(const Vector2D& normal) const;

		// ������������֮��Ĳ�ֵ���������Բ�ֵ��
		Vector2D interpolate(const Vector2D& other, float t) const;

		// ������ת��Ϊ��������ʽ�����ؼ����ͼ��ǣ�
		struct PolarCoordinates {
			float radius;
			float angle;
		};
		PolarCoordinates toPolar() const;

		// ��������������ʾ�⣬����ʹ�ö�ά���������ʵ�ʿ�ʹ�ø�ͨ�õ�Gram-Schmidt���̣�
		Vector2D orthogonalize() const;

		// �������������
		Vector2D linearCombination(float scalar1, const Vector2D& v1, float scalar2, const Vector2D& v2) const;
	};

	// ���ر����˷��������������������ߵ������
	Vector2D operator*(float scalar, const Vector2D& vector);
}