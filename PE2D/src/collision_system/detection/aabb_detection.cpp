#include "collision_system/detection/aabb_detection.hpp"
#include "types/common_types.hpp"
//四叉树
namespace PE2D {
	// 向当前节点插入一个 AABB
	void QuadTreeNode::insert(const AABB& aabb) {
		// 如果当前节点已经分裂，尝试将 AABB 插入到合适的子节点中
		if (m_nodes[0] != nullptr) {
			int index = getIndex(aabb);
			if (index != -1) {
				m_nodes[index]->insert(aabb);
				return;
			}
		}

		// 如果不适合插入到子节点中，将 AABB 存储在当前节点
		m_objects.push_back(aabb.getObj_ID());

		// 如果当前节点存储的 AABB 数量超过最大允许数量，且未达到最大层级，则进行分裂
		if (m_objects.size() > m_maxObjects && m_level < m_maxLevel) {
			if (m_nodes[0] == nullptr) {
				split();
			}

			int i = 0;
			while (i < m_objects.size()) {
				int index = getIndex(*Object::ID_Map[m_objects[i]]->getAABB());
				if (index != -1) {
					// 如果 AABB 适合插入到子节点中，将其从当前节点移除并插入到子节点
					m_nodes[index]->insert(*Object::ID_Map[m_objects[i]]->getAABB());
					m_objects.erase(m_objects.begin() + i);
				}
				else {
					i++;
				}
			}
		}
	}

	// 检索与指定 AABB 可能相交的所有 AABB 的索引
	std::vector<unsigned int> QuadTreeNode::retrieve(const AABB& aabb) {
		std::vector<unsigned int> returnObjects;
		int index = getIndex(aabb);
		// 如果 AABB 适合某个子节点，递归地从该子节点检索可能相交的 AABB
		if (index != -1 && m_nodes[0] != nullptr) {
			std::vector<unsigned int> childObjects = m_nodes[index]->retrieve(aabb);
			returnObjects.insert(returnObjects.end(), childObjects.begin(), childObjects.end());
		}

		// 将当前节点存储的所有 AABB 添加到结果中
		returnObjects.insert(returnObjects.end(), m_objects.begin(), m_objects.end());
		return returnObjects;
	}

	// 将当前节点分裂为四个子节点
	void QuadTreeNode::split() {
		float subWidth = m_bounds.getWidth() / 2;
		float subHeight = m_bounds.getHeight() / 2;
		float x = m_bounds.getTopLeft().x();
		float y = m_bounds.getTopLeft().y();

		// 创建四个子节点，分别代表当前节点区域的四个象限
		m_nodes[0] = new QuadTreeNode(AABB(Vector2D(x + subWidth, y), Vector2D(x + 2 * subWidth, y + subHeight)), m_level + 1, m_maxLevel, m_maxObjects);
		m_nodes[1] = new QuadTreeNode(AABB(Vector2D(x, y), Vector2D(x + subWidth, y + subHeight)), m_level + 1, m_maxLevel, m_maxObjects);
		m_nodes[2] = new QuadTreeNode(AABB(Vector2D(x, y + subHeight), Vector2D(x + subWidth, y + 2 * subHeight)), m_level + 1, m_maxLevel, m_maxObjects);
		m_nodes[3] = new QuadTreeNode(AABB(Vector2D(x + subWidth, y + subHeight), Vector2D(x + 2 * subWidth, y + 2 * subHeight)), m_level + 1, m_maxLevel, m_maxObjects);
	}

	// 获取指定 AABB 应该插入的子节点索引
	int QuadTreeNode::getIndex(const AABB& aabb) {
		int index = -1;
		float verticalMidpoint = m_bounds.getTopLeft().x() + m_bounds.getWidth() / 2;
		float horizontalMidpoint = m_bounds.getTopLeft().y() + m_bounds.getHeight() / 2;

		// 判断 AABB 位于当前节点区域的哪个象限
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