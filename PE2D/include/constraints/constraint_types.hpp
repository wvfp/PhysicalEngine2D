#pragma once
#include "types/common_types.hpp"
#include <vector>
#include <memory>

namespace PE2D {
    // Լ������
    class Constraint {
        Constraint(const Constraint&) = delete; // Disable copy constructor ���ø��ƹ��캯��
        Constraint(Constraint&&) = delete; // Disable move constructor �����ƶ����캯��
        Constraint& operator=(const Constraint&) = delete; // Disable copy assignment operator ���ø��Ƹ�ֵ�����
        Constraint& operator=(Constraint&&) = delete; // Disable move assignment operator �����ƶ���ֵ�����
    public:
        //����ID����������֮���Լ��
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
        // ��ʼ��Լ��
        virtual void initialize(void*args) {
            // ��ʼ���߼�
        }
        // ����Լ��
        virtual void update(float deltaTime) {
            // �����߼�
            /*
            
            if(m_isActive){
            ...
            }
            */
        }
        // Ӧ��Լ��
        virtual void apply() {
            // ����Լ��Ӧ���߼�
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
        //Լ���Ƿ񼤻�
        bool isActive() { return m_isActive; }
        //Լ��״̬���ã�true ���false ����
        void setActive(bool b) { m_isActive=b; }
    protected:
        unsigned int m_obj1; // ��һ������
        unsigned int m_obj2; // �ڶ�������
    public:
        static std::vector<std::shared_ptr<Constraint>> ID_Map;
    private:
        unsigned int m_ID;
        static unsigned int m_nextID;
        bool m_isActive = true;
    };

}