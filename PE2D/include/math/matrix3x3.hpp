#pragma once
#include "math/vector2d.hpp"
#include <stdexcept>
namespace PE2D {
	class Matrix3x3 {
	private:
		float data[3][3];

	public:
		// Ĭ�Ϲ��캯������ʼ��Ϊ��λ����
		Matrix3x3();

		// ��ֵ���캯�����Խ�Ԫ�س�ʼ��Ϊ��ֵ������Ϊ 0
		explicit Matrix3x3(float value);

		// 9 ��ֵ���캯��
		Matrix3x3(float a11, float a12, float a13,
			float a21, float a22, float a23,
			float a31, float a32, float a33);

		// ���鹹�캯��
		explicit Matrix3x3(const float arr[9]);

		// 3 ���������� 3 �����飩���캯��
		Matrix3x3(const float* row1, const float* row2, const float* row3);

		// ���ؼӷ������
		Matrix3x3 operator+(const Matrix3x3& other) const;

		// ���ؼ��������
		Matrix3x3 operator-(const Matrix3x3& other) const;

		// ���س˷��������������ˣ�
		Matrix3x3 operator*(const Matrix3x3& other) const;

		// ����
		Matrix3x3 operator*(float scalar) const;
		// ��һ��2ά���������
		Vector2D operator*(const Vector2D& v)const;
		// ת��
		Matrix3x3 transpose() const;
		// ���Ų���
		static Matrix3x3 scale(float sx, float sy);
		static Matrix3x3 scale(float s) { return scale(s, s); };
		// ������ʽ
		float determinant() const;

		// ��ӡ����
		void print() const;

		// ��ת����
		static Matrix3x3 rotate(float angle);

		// ƽ�Ʋ���
		static Matrix3x3 translate(float tx, float ty);

		// ��ȡԪ��
		float& operator()(int row, int col);
		const float& operator()(int row, int col) const;
	};
}