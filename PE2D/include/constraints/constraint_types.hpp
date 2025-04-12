#pragma once
#include "types/common_types.hpp"
#include <vector>
#include <memory>

namespace PE2D {
    // 约束基类
    class Constraint {
        Constraint(const Constraint&) = delete; // Disable copy constructor 禁用复制构造函数
        Constraint(Constraint&&) = delete; // Disable move constructor 禁用移动构造函数
        Constraint& operator=(const Constraint&) = delete; // Disable copy assignment operator 禁用复制赋值运算符
        Constraint& operator=(Constraint&&) = delete; // Disable move assignment operator 禁用移动赋值运算符
    public:
        //根据ID来建立物体之间的约束
        Constraint(const unsigned int& obj1, const unsigned int& obj2)
            : m_obj1(obj1), m_obj2(obj2),m_ID(m_nextID) {
            // Add the object to the ID map
            if (m_ID == ID_Map.size())
                ID_Map.push_back(std::shared_ptr<Constraint>(this));
            else
                ID_Map[m_ID] = std::shared_ptr<Constraint>(this);
            m_nextID = getNextID();
        }
        virtual ~Constraint() { removeFromMap(m_ID); };
        // 初始化约束
        virtual void initialize(void*args) {
            // 初始化逻辑
        }
        // 更新约束
        virtual void update(float deltaTime) {
            // 更新逻辑
            /*
            
            if(m_isActive){
            ...
            }
            */
        }
        // 应用约束
        virtual void apply() {
            // 基本约束应用逻辑
            /*
            if (m_isActive) {
                ...
            }
            */
        }
        unsigned int getID() { return m_ID; }
        unsigned int getNextID()const;
        static void removeFromMap(unsigned int);
        static bool isValidID(unsigned int);
        //约束是否激活
        bool isActive() { return m_isActive; }
        //约束状态设置，true 激活，false 休眠
        void setActive(bool b) { m_isActive=b; }
    protected:
        unsigned int m_obj1; // 第一个物体
        unsigned int m_obj2; // 第二个物体
    public:
        static std::vector<std::shared_ptr<Constraint>> ID_Map;
    private:
        unsigned int m_ID;
        static unsigned int m_nextID;
        bool m_isActive = true;
    };

}