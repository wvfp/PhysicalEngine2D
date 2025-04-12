#pragma once
#include "math/matrix3x3.hpp"
#include "math/math_utils.hpp"

namespace PE2D{
	class Rotation {
		static Matrix3x3 calculateRotation(Vector2D angular,float deltaTime) {
			// ¼ÆËãĞı×ª¾ØÕó
			float angle = angular.magnitude() * deltaTime;
			return Matrix3x3::rotate(angle);
		}
	};
}