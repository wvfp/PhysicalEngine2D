#pragma once
#include <iostream>
#include <chrono>
#include <thread>
namespace PE2D {
    class Timer {
    public:
        Timer() {
            // 初始化时记录开始时间
            reset();
        }
        // 重置计时器
        void reset() {
            m_startTime = std::chrono::high_resolution_clock::now();
        }
        // 获取自上次重置以来的时间（秒）
        float getElapsedTime() const {
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> duration = now - m_startTime;
            return duration.count();
        }
        // 获取时间步长（自上次调用此方法以来的时间）
        float getDeltaTime() {
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> duration = now - m_lastFrameTime;
            m_lastFrameTime = now;
            return duration.count();
        }
        // 控制帧率（每秒固定帧数）
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
