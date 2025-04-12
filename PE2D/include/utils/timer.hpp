#pragma once
#include <iostream>
#include <chrono>
#include <thread>
namespace PE2D {
    class Timer {
    public:
        Timer() {
            // ��ʼ��ʱ��¼��ʼʱ��
            reset();
        }
        // ���ü�ʱ��
        void reset() {
            m_startTime = std::chrono::high_resolution_clock::now();
        }
        // ��ȡ���ϴ�����������ʱ�䣨�룩
        float getElapsedTime() const {
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> duration = now - m_startTime;
            return duration.count();
        }
        // ��ȡʱ�䲽�������ϴε��ô˷���������ʱ�䣩
        float getDeltaTime() {
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> duration = now - m_lastFrameTime;
            m_lastFrameTime = now;
            return duration.count();
        }
        // ����֡�ʣ�ÿ��̶�֡����
        void capFrameRate(float targetFPS) {
            if (targetFPS <= 0.0f) {
                return;
            }
            float frameTime = 1.0f / targetFPS;
            float deltaTime = getDeltaTime();
            if (deltaTime < frameTime) {
                std::this_thread::sleep_for(std::chrono::duration<float>(frameTime - deltaTime));
            }
        }
        void capFrameRateWithDeltaTime(float frameTime) {
            float deltaTime = getDeltaTime();
            if (deltaTime < frameTime) {
				std::this_thread::sleep_for(std::chrono::duration<float>(frameTime - deltaTime));
            }
        }
    private:
        std::chrono::high_resolution_clock::time_point m_startTime;
        std::chrono::high_resolution_clock::time_point m_lastFrameTime;
    };

}
