#pragma once
#include "collision_system/collision_types.hpp"
#include <vector>
#include <math/vector2d.hpp>
#include <math/geometry2d.hpp>
#include "types/common_types.hpp"

namespace PE2D {
	// �Ĳ����ڵ��࣬���ڱ�ʾ�Ĳ����е�һ���ڵ�
	class QuadTreeNode {
	public:
		// ���캯������ʼ���Ĳ����ڵ�
		// bounds: ��ǰ�ڵ�������� AABB ����
		// level: ��ǰ�ڵ����ڵĲ㼶
		// maxLevel: �Ĳ�����������㼶
		// maxObjects: ÿ���ڵ�����洢����� AABB ����
		QuadTreeNode(const AABB& bounds, int level, int maxLevel, int maxObjects)
			: m_bounds(bounds), m_level(level), m_maxLevel(maxLevel), m_maxObjects(maxObjects) {
		}

		// ��ǰ�ڵ����һ�� AABB
		// aabb: Ҫ����� AABB
		void insert(const AABB& aabb);

		// ������ָ�� AABB �����ཻ������ AABB ������
		// aabb: ���ڼ����� AABB
		// ����ֵ: ���������ཻ�� AABB ����������
		std::vector<unsigned> retrieve(const AABB& aabb);

	private:
		// ����ǰ�ڵ����Ϊ�ĸ��ӽڵ�
		void split();

		// ��ȡָ�� AABB Ӧ�ò�����ӽڵ�����
		// aabb: Ҫ����� AABB
		// ����ֵ: �ӽڵ��������������ʺ��κ��ӽڵ��򷵻� -1
		int getIndex(const AABB& aabb);

		// �洢��ǰ�ڵ���������� AABB
		std::vector<unsigned> m_objects;
		// ��ǰ�ڵ�������� AABB ����
		AABB m_bounds;
		// �ĸ��ӽڵ��ָ������
		QuadTreeNode* m_nodes[4] = { nullptr };
		// ��ǰ�ڵ����ڵĲ㼶
		int m_level;
		// �Ĳ�����������㼶
		int m_maxLevel;
		// ÿ���ڵ�����洢����� AABB ����
		int m_maxObjects;
	};

	// �Ĳ����࣬���ڹ����Ĳ����ṹ
	class QuadTree {
	public:
		// ���캯������ʼ���Ĳ���
		// bounds: �Ĳ������ڵ�������� AABB ����
		// maxLevel: �Ĳ�����������㼶��Ĭ��Ϊ 5
		// maxObjects: ÿ���ڵ�����洢����� AABB ������Ĭ��Ϊ 10
		QuadTree(const AABB& bounds, int maxLevel = 5, int maxObjects = 10)
			: m_root(bounds, 0, maxLevel, maxObjects) {
		}
		// ����һ��AABB���õ�AABB�������Ŀռ䷶Χ
		static AABB getRootbyAABBS(std::vector<unsigned int> ids) {
			float max_x=0, max_y=0, min_x=0, min_y=0;
			for (auto id : ids) {
				Vector2D br = Object::ID_Map[id]->getAABB()->getBottomRight();
				Vector2D tf = Object::ID_Map[id]->getAABB()->getTopLeft();
				min_x = min_x > br.x() ? br.x():min_x;
				min_y = min_y > br.y() ? br.y():min_y;
				max_x = max_x < tf.x() ? tf.x() : max_x;
				max_y = max_y < tf.y() ? tf.y() : max_y;
			}
			AABB a=AABB(Vector2D(min_x, min_y), Vector2D(max_x, max_y));
			a.setObj_ID(0);
			return a;
		}

		// ���Ĳ����в���һ�� AABB
		// aabb: Ҫ����� AABB
		void insert(const AABB& aabb) {
			m_root.insert(aabb);
		}

		// ���Ĳ����м�����ָ�� AABB �����ཻ������ AABB ������
		// aabb: ���ڼ����� AABB
		// ����ֵ: ���������ཻ�� AABB ����������
		std::vector<unsigned int> retrieve(const AABB& aabb) {
			return m_root.retrieve(aabb);
		}

	private:
		// �Ĳ����ĸ��ڵ�
		QuadTreeNode m_root;
	};

	// ���񻮷��࣬���ڽ��ռ仮��Ϊ����Ԫ
	class Grid {
	public:
		// ���캯������ʼ������
		// bounds: ���������ǵ� AABB ����
		// cellSize: ÿ������Ԫ�Ĵ�С
		Grid(const AABB& bounds, int cellSize)
			: m_bounds(bounds), m_cellSize(cellSize) {
			int width = static_cast<int>(bounds.getWidth() / cellSize);
			int height = static_cast<int>(bounds.getHeight() / cellSize);
			// ��ʼ������Ԫ����
			m_cells.resize(width * height);
		}

		// �������в���һ�� AABB
		// aabb: Ҫ����� AABB
		void insert(const AABB& aabb) {
			// ���� AABB ���ǵ���ʼ�ͽ�������Ԫ����
			int startX = static_cast<int>((aabb.getTopLeft().x() - m_bounds.getTopLeft().x()) / m_cellSize);
			int startY = static_cast<int>((aabb.getTopLeft().y() - m_bounds.getTopLeft().y()) / m_cellSize);
			int endX = static_cast<int>((aabb.getBottomRight().x() - m_bounds.getTopLeft().x()) / m_cellSize);
			int endY = static_cast<int>((aabb.getBottomRight().y() - m_bounds.getTopLeft().y()) / m_cellSize);

			// �� AABB ���뵽�串�ǵ���������Ԫ��
			for (int y = startY; y <= endY; ++y) {
				for (int x = startX; x <= endX; ++x) {
					int index = y * static_cast<int>(m_bounds.getWidth() / m_cellSize) + x;
					if (index >= 0 && index < m_cells.size()) {
						m_cells[index].push_back(aabb);
					}
				}
			}
		}

		// �������м�����ָ�� AABB �����ཻ������ AABB
		// aabb: ���ڼ����� AABB
		// ����ֵ: ���������ཻ�� AABB ������
		std::vector<AABB> retrieve(const AABB& aabb) {
			std::vector<AABB> result;
			// ���� AABB ���ǵ���ʼ�ͽ�������Ԫ����
			int startX = static_cast<int>((aabb.getTopLeft().x() - m_bounds.getTopLeft().x()) / m_cellSize);
			int startY = static_cast<int>((aabb.getTopLeft().y() - m_bounds.getTopLeft().y()) / m_cellSize);
			int endX = static_cast<int>((aabb.getBottomRight().x() - m_bounds.getTopLeft().x()) / m_cellSize);
			int endY = static_cast<int>((aabb.getBottomRight().y() - m_bounds.getTopLeft().y()) / m_cellSize);

			// �� AABB ���ǵ���������Ԫ���ռ������ཻ�� AABB
			for (int y = startY; y <= endY; ++y) {
				for (int x = startX; x <= endX; ++x) {
					int index = y * static_cast<int>(m_bounds.getWidth() / m_cellSize) + x;
					if (index >= 0 && index < m_cells.size()) {
						result.insert(result.end(), m_cells[index].begin(), m_cells[index].end());
					}
				}
			}
			return result;
		}

	private:
		// ���������ǵ� AABB ����
		AABB m_bounds;
		// ÿ������Ԫ�Ĵ�С
		int m_cellSize;
		// �洢��������Ԫ�Ķ�ά����
		std::vector<std::vector<AABB>> m_cells;
	};
}