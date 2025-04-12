#include "collision_system/detection/aabb_detection.hpp"
#include "types/common_types.hpp"
//�Ĳ���
namespace PE2D {
	// ��ǰ�ڵ����һ�� AABB
	void QuadTreeNode::insert(const AABB& aabb) {
		// �����ǰ�ڵ��Ѿ����ѣ����Խ� AABB ���뵽���ʵ��ӽڵ���
		if (m_nodes[0] != nullptr) {
			int index = getIndex(aabb);
			if (index != -1) {
				m_nodes[index]->insert(aabb);
				return;
			}
		}

		// ������ʺϲ��뵽�ӽڵ��У��� AABB �洢�ڵ�ǰ�ڵ�
		m_objects.push_back(aabb.getObj_ID());

		// �����ǰ�ڵ�洢�� AABB �����������������������δ�ﵽ���㼶������з���
		if (m_objects.size() > m_maxObjects && m_level < m_maxLevel) {
			if (m_nodes[0] == nullptr) {
				split();
			}

			int i = 0;
			while (i < m_objects.size()) {
				int index = getIndex(*Object::ID_Map[m_objects[i]]->getAABB());
				if (index != -1) {
					// ��� AABB �ʺϲ��뵽�ӽڵ��У�����ӵ�ǰ�ڵ��Ƴ������뵽�ӽڵ�
					m_nodes[index]->insert(*Object::ID_Map[m_objects[i]]->getAABB());
					m_objects.erase(m_objects.begin() + i);
				}
				else {
					i++;
				}
			}
		}
	}

	// ������ָ�� AABB �����ཻ������ AABB ������
	std::vector<unsigned int> QuadTreeNode::retrieve(const AABB& aabb) {
		std::vector<unsigned int> returnObjects;
		int index = getIndex(aabb);
		// ��� AABB �ʺ�ĳ���ӽڵ㣬�ݹ�شӸ��ӽڵ���������ཻ�� AABB
		if (index != -1 && m_nodes[0] != nullptr) {
			std::vector<unsigned int> childObjects = m_nodes[index]->retrieve(aabb);
			returnObjects.insert(returnObjects.end(), childObjects.begin(), childObjects.end());
		}

		// ����ǰ�ڵ�洢������ AABB ��ӵ������
		returnObjects.insert(returnObjects.end(), m_objects.begin(), m_objects.end());
		return returnObjects;
	}

	// ����ǰ�ڵ����Ϊ�ĸ��ӽڵ�
	void QuadTreeNode::split() {
		float subWidth = m_bounds.getWidth() / 2;
		float subHeight = m_bounds.getHeight() / 2;
		float x = m_bounds.getTopLeft().x();
		float y = m_bounds.getTopLeft().y();

		// �����ĸ��ӽڵ㣬�ֱ����ǰ�ڵ�������ĸ�����
		m_nodes[0] = new QuadTreeNode(AABB(Vector2D(x + subWidth, y), Vector2D(x + 2 * subWidth, y + subHeight)), m_level + 1, m_maxLevel, m_maxObjects);
		m_nodes[1] = new QuadTreeNode(AABB(Vector2D(x, y), Vector2D(x + subWidth, y + subHeight)), m_level + 1, m_maxLevel, m_maxObjects);
		m_nodes[2] = new QuadTreeNode(AABB(Vector2D(x, y + subHeight), Vector2D(x + subWidth, y + 2 * subHeight)), m_level + 1, m_maxLevel, m_maxObjects);
		m_nodes[3] = new QuadTreeNode(AABB(Vector2D(x + subWidth, y + subHeight), Vector2D(x + 2 * subWidth, y + 2 * subHeight)), m_level + 1, m_maxLevel, m_maxObjects);
	}

	// ��ȡָ�� AABB Ӧ�ò�����ӽڵ�����
	int QuadTreeNode::getIndex(const AABB& aabb) {
		int index = -1;
		float verticalMidpoint = m_bounds.getTopLeft().x() + m_bounds.getWidth() / 2;
		float horizontalMidpoint = m_bounds.getTopLeft().y() + m_bounds.getHeight() / 2;

		// �ж� AABB λ�ڵ�ǰ�ڵ�������ĸ�����
		bool topQuadrant = (aabb.getTopLeft().y() < horizontalMidpoint && aabb.getBottomRight().y() < horizontalMidpoint);
		bool bottomQuadrant = (aabb.getTopLeft().y() > horizontalMidpoint);

		if (aabb.getTopLeft().x() < verticalMidpoint && aabb.getBottomRight().x() < verticalMidpoint) {
			if (topQuadrant) {
				index = 1;
			}
			else if (bottomQuadrant) {
				index = 2;
			}
		}
		else if (aabb.getTopLeft().x() > verticalMidpoint) {
			if (topQuadrant) {
				index = 0;
			}
			else if (bottomQuadrant) {
				index = 3;
			}
		}

		return index;
	}
}