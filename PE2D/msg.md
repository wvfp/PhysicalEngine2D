����һ��2D�����������Ŀ�ṹ�������Ƕ����и����ļ����õ���ϸ���ͣ�

1. **include/Ŀ¼**�����ͷ�ļ��������ࡢ�������ṹ��ȣ�������Դ�ļ�����ʹ�á�
    - **2DPhysicsEngine.hpp**�������������ͷ�ļ������ܰ���������ģ���ͷ�ļ�������ͳһ���롣
    - **math/Ŀ¼**����ѧ��ص�ͷ�ļ���
        - **algebra/Ŀ¼**��������أ���`vector2d.hpp`�����ά������`matrix3x3.hpp`����3x3����
        - **geometry/Ŀ¼**��������أ�`point2d.hpp`�����ά�㣬`line2d.hpp`�����άֱ�ߡ�
        - **math_types.hpp**��������ѧ��ص��������͡�
        - **math_utils.hpp**��������ѧ���ߺ�����
    - **entities/Ŀ¼**��ʵ����ص�ͷ�ļ���
        - **base_entity.hpp**���������ʵ���ࡣ
        - **rigidbody.hpp**����������ࡣ
        - **collider.hpp**��������ײ���ࡣ
        - **entity_types.hpp**������ʵ����ص��������͡�
    - **collision_system/Ŀ¼**����ײϵͳ��ص�ͷ�ļ���
        - **detection/Ŀ¼**����ײ�����أ���`aabb_detection.hpp`����������Χ�м�⣬`circle_detection.hpp`����Բ�μ�⡣
        - **resolution/Ŀ¼**����ײ������أ�`penetration_resolution.hpp`����͸���⣬`impulse_resolution.hpp`����������⡣
        - **collision_types.hpp**��������ײ��ص��������͡�
    - **motion_system/Ŀ¼**���˶�ϵͳ��ص�ͷ�ļ���
        - **integrators/Ŀ¼**����������أ�`euler_integrator.hpp`ʵ��ŷ����������`verlet_integrator.hpp`ʵ��Verlet��������
        - **forces/Ŀ¼**������أ�`gravity_force.hpp`����������`spring_force.hpp`���嵯������
        - **motion_types.hpp**�������˶���ص��������͡�
    - **config/Ŀ¼**��������ص�ͷ�ļ���`engine_config.hpp`����������������Ĳ�����
    - **utils/Ŀ¼**��������ص�ͷ�ļ���
        - **logger.hpp**����־��¼��ء�
        - **timer.hpp**����ʱ����ء�
        - **memory_manager.hpp**���ڴ������ء�
        - **error_handling.hpp**����������ء�
        - **assertions.hpp**��������ء�
        - **utils_macros.hpp**�����ߺ궨�塣
    - **types/Ŀ¼**�����ͨ�����Ͷ����ͷ�ļ���`common_types.hpp`����ͨ�õ��������͡�

2. **src/Ŀ¼**�����Դ�ļ���ʵ��ͷ�ļ��������Ĺ��ܡ�
    - **2DPhysicsEngine.cpp**�������������Դ�ļ������ܰ�����ʼ�������ٵȹ��ܡ�
    - **math/��entities/��collision_system/��motion_system/��config/��utils/��types/Ŀ¼�µ�Դ�ļ�**��ʵ�ֶ�Ӧͷ�ļ��������ĺ�������ľ����߼���

3. **CMakeLists.txt**��CMake�����ű������ڶ�����Ŀ�Ĺ���������Դ�ļ���ͷ�ļ�·�������ӿ�ȡ�

4. **LICENSE**����Ŀ�����֤�ļ���˵����Ŀ��ʹ�á��ַ���Ȩ�ޡ�

5. **README.md**����Ŀ��˵���ĵ���������Ŀ�Ĺ��ܡ�ʹ�÷�������������Ϣ�� 