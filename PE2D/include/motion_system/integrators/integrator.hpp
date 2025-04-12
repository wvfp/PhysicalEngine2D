// integrator.hpp
#pragma once
#include "motion_system/motion_types.hpp"

namespace PE2D {

    // ��ʽŷ�����������Ż��棩
    class ExplicitEulerIntegration : public IntegrationMethod {
    public:
        Parameter operator()(const Parameter& params) override {
            Parameter newParams = params;

            // Ԥ���㵹�������ظ�����
            const float inv_mass = 1.0 / newParams.mass;
            const float inv_inertia = 1.0 / newParams.inertia;

            // ���¼��ٶ�
            newParams.acceleration = newParams.force * inv_mass;
            newParams.angular_acceleration = newParams.torque * inv_inertia;

            // �����ٶȣ��ϲ�������
            newParams.velocity = newParams.velocity +
                (newParams.acceleration * newParams.delta_time) +
                (newParams.impulse * inv_mass);

            // ���½��ٶ�
            newParams.angular_velocity += newParams.angular_acceleration * newParams.delta_time;

            // ����λ��
            newParams.position = newParams.position +
                newParams.velocity * newParams.delta_time;

            return newParams;
        }
    };

    // ����ʽŷ�����������Ż��棩
    class SemiImplicitEulerIntegration : public IntegrationMethod {
    public:
        Parameter operator()(const Parameter& params) override {
            Parameter newParams = params;
            const float inv_mass = 1.0 / newParams.mass;
            const float inv_inertia = 1.0 / newParams.inertia;

            newParams.acceleration = newParams.force * inv_mass;
            newParams.angular_acceleration = newParams.torque * inv_inertia;

            // �ȸ����ٶ�
            newParams.velocity = newParams.velocity +
                (newParams.acceleration * newParams.delta_time) +
                (newParams.impulse * inv_mass);

            newParams.angular_velocity += newParams.angular_acceleration * newParams.delta_time;

            // �����ٶȸ���λ��
            newParams.position = newParams.position +
                newParams.velocity * newParams.delta_time;

            return newParams;
        }
    };

    // ţ�ٵ������������������㣩
    class NewtonRaphsonIteration : public IterationMethod {
    public:
        Vector2D operator()(const Vector2D& initialGuess, const Parameter& params) override {
            const int maxIterations = 100;
            const float tolerance = 1e-6;
            const float inv_mass = 1.0 / params.mass;
            const float dt_damping = params.delta_time * params.damping * inv_mass;

            Vector2D currentGuess = initialGuess;

            for (int i = 0; i < maxIterations; ++i) {
                Vector2D functionValue = currentGuess - params.velocity -
                    (params.force * inv_mass +
                        currentGuess * dt_damping) * params.delta_time;

                // ����������������ÿ��������������
                float derivativeX = 1.0 - dt_damping;
                float derivativeY = 1.0 - dt_damping;

                if (functionValue.magnitude() < tolerance) break;

                currentGuess.setX( currentGuess.x() - functionValue.x() / derivativeX);
                currentGuess.setY(currentGuess.y() - functionValue.y() / derivativeY);
            }

            return currentGuess;
        }
    };

    // ��ʽŷ�����������Ż��棩
    class ImplicitEulerIntegration : public IntegrationMethod {
        IterationMethod& iterationMethod;
    public:
        ImplicitEulerIntegration(IterationMethod& method) : iterationMethod(method) {}

        Parameter operator()(const Parameter& params) override {
            Parameter newParams = params;
            const float inv_mass = 1.0 / newParams.mass;
            const float inv_inertia = 1.0 / newParams.inertia;

            // ����������ٶ�
            Vector2D new_velocity = iterationMethod(newParams.velocity, newParams);

            // ����������
            newParams.acceleration = (newParams.force - new_velocity * newParams.damping) * inv_mass;
            newParams.angular_acceleration = newParams.torque * inv_inertia;

            newParams.position = newParams.position + new_velocity * newParams.delta_time;
            newParams.velocity = new_velocity + newParams.impulse * inv_mass;
            newParams.angular_velocity += newParams.angular_acceleration * newParams.delta_time;

            return newParams;
        }
    };

    // �Ľ�����-���������Ż��棩
    class RungeKuttaIntegration : public IntegrationMethod {
    public:
        Parameter operator()(const Parameter& params) override {
            Parameter newParams = params;
            const float inv_mass = 1.0 / newParams.mass;
            const float inv_inertia = 1.0 / newParams.inertia;
            const float half_dt = newParams.delta_time * 0.5;

            auto calcAccel = [&](const Vector2D& f) { return f * inv_mass; };
            auto calcAngularAccel = [&](float t) { return t * inv_inertia; };

            // �ٶ�RK4
            Vector2D k1_v = calcAccel(newParams.force);
            Vector2D k2_v = calcAccel(newParams.force);
            Vector2D k3_v = calcAccel(newParams.force);
            Vector2D k4_v = calcAccel(newParams.force);

            // λ��RK4
            Vector2D k1_r = newParams.velocity;
            Vector2D k2_r = newParams.velocity + k1_v * half_dt;
            Vector2D k3_r = newParams.velocity + k2_v * half_dt;
            Vector2D k4_r = newParams.velocity + k3_v * newParams.delta_time;

            // ���ٶ�RK4
            float k1_omega = calcAngularAccel(newParams.torque.magnitude());
            float k2_omega = calcAngularAccel(newParams.torque.magnitude());
            float k3_omega = calcAngularAccel(newParams.torque.magnitude());
            float k4_omega = calcAngularAccel(newParams.torque.magnitude());

            // ����״̬
            newParams.velocity = newParams.velocity +
                (k1_v + k2_v * 2 + k3_v * 2 + k4_v) * (newParams.delta_time / 6.0) +
                newParams.impulse * inv_mass;

            newParams.position = newParams.position +
                (k1_r + k2_r * 2 + k3_r * 2 + k4_r) * (newParams.delta_time / 6.0);

            newParams.angular_velocity += (k1_omega + 2 * k2_omega + 2 * k3_omega + k4_omega) *
                (newParams.delta_time / 6.0);

            return newParams;
        }
    };

    // Verlet�������������棩
    class VerletIntegration : public IntegrationMethod {
    public:
        Parameter operator()(const Parameter& params) override {
            Parameter newParams = params;

            // ���㵱ǰ���ٶ�
            newParams.acceleration = newParams.force / newParams.mass;

            // Verletλ�ø���
            const Vector2D currentPos = newParams.position;
            newParams.position = 2 * newParams.position - newParams.prev_position +
                newParams.acceleration * newParams.delta_time * newParams.delta_time;
            newParams.prev_position = currentPos;

            // �����ٶ�
            newParams.velocity = (newParams.position - newParams.prev_position) / newParams.delta_time +
                newParams.impulse / newParams.mass;

            // ���½��ٶ�
            newParams.angular_acceleration = newParams.torque / newParams.inertia;
            newParams.angular_velocity += newParams.angular_acceleration * newParams.delta_time;

            return newParams;
        }
    };
} // namespace PE2D