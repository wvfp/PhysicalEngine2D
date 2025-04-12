#pragma once
#include "math/math_utils.hpp"
#include "math/vector2d.hpp"
#include "math/matrix3x3.hpp"
#include "math/geometry2d.hpp"
#include <functional>
#include <algorithm>

#include <iostream>

namespace PE2D {
	//Integration methods
	typedef struct {
		Vector2D position;//λ��
		Vector2D velocity;//�ٶ�
		Vector2D acceleration;//���ٶ�
		Vector2D force;//��
		Vector2D impulse;//����
		Vector2D torque;//����
		Vector2D angular_velocity;//���ٶ�
		Vector2D angular_acceleration;//�Ǽ��ٶ�
		float mass;//����
		float inertia;//ת������
		float friction;//Ħ��ϵ��
		float damping;//����ϵ��
		float angularDamping;
		float restitution;//����ϵ��
		float delta_time;//ʱ�䲽��
		// ��¼��һʱ�̵�λ�ã����� Verlet ���ֵ�
		Vector2D prev_position;
		// �������״
		Shape* shape;
	}Parameter;
	// ���ַ�������
	class IntegrationMethod {
	public:
		//���ַ�����һ�����⺯��operator()������һ��Parameter���͵Ĳ���������һ��Parameter���͵Ľ��
		//��������������Ƕ�����ϵͳ���л��ּ��㣬����λ�á��ٶȵȲ���
		//����Ļ��ַ���������ʵ��
		virtual Parameter operator()(const Parameter& params) = 0;
		virtual ~IntegrationMethod() = default;
	};

	// ������������
	class IterationMethod {
	public:
		//��������
		virtual Vector2D operator()(const Vector2D& initialGuess, const Parameter& params) = 0;
		virtual Vector2D operator()(const Vector2D& initialGuess1, const Vector2D& initialGuess2, const Parameter& params) {};
		virtual ~IterationMethod() = default;
	};
}