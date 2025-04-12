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
		Vector2D position;//位置
		Vector2D velocity;//速度
		Vector2D acceleration;//加速度
		Vector2D force;//力
		Vector2D impulse;//冲量
		Vector2D torque;//力矩
		Vector2D angular_velocity;//角速度
		Vector2D angular_acceleration;//角加速度
		float mass;//质量
		float inertia;//转动惯量
		float friction;//摩擦系数
		float damping;//阻尼系数
		float angularDamping;
		float restitution;//弹性系数
		float delta_time;//时间步长
		// 记录上一时刻的位置，用于 Verlet 积分等
		Vector2D prev_position;
		// 物体的形状
		Shape* shape;
	}Parameter;
	// 积分方法基类
	class IntegrationMethod {
	public:
		//积分方法，一个虚拟函数operator()，接受一个Parameter类型的参数，返回一个Parameter类型的结果
		//这个函数的作用是对物理系统进行积分计算，更新位置、速度等参数
		//具体的积分方法由子类实现
		virtual Parameter operator()(const Parameter& params) = 0;
		virtual ~IntegrationMethod() = default;
	};

	// 迭代方法基类
	class IterationMethod {
	public:
		//迭代方法
		virtual Vector2D operator()(const Vector2D& initialGuess, const Parameter& params) = 0;
		virtual Vector2D operator()(const Vector2D& initialGuess1, const Vector2D& initialGuess2, const Parameter& params) {};
		virtual ~IterationMethod() = default;
	};
}