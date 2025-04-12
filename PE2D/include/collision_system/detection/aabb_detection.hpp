#pragma once
#include "collision_system/collision_types.hpp"
#include <vector>
#include <math/vector2d.hpp>
#include <math/geometry2d.hpp>
#include "types/common_types.hpp"

namespace PE2D {
	// 四叉树节点类，用于表示四叉树中的一个节点
	class QuadTreeNode {
	public:
		// 构造函数，初始化四叉树节点
		// bounds: 当前节点所代表的 AABB 区域
		// level: 当前节点所在的层级
		// maxLevel: 四叉树允许的最大层级
		// maxObjects: 每个节点允许存储的最大 AABB 数量
		QuadTreeNode(const AABB& bounds, int level, int maxLevel, int maxObjects)
			: m_bounds(bounds), m_level(level), m_maxLevel(maxLevel), m_maxObjects(maxObjects) {
		}

		// 向当前节点插入一个 AABB
		// aabb: 要插入的 AABB
		void insert(const AABB& aabb);

		// 检索与指定 AABB 可能相交的所有 AABB 的索引
		// aabb: 用于检索的 AABB
		// 返回值: 包含可能相交的 AABB 索引的向量
		std::vector<unsigned> retrieve(const AABB& aabb);

	private:
		// 将当前节点分裂为四个子节点
		void split();

		// 获取指定 AABB 应该插入的子节点索引
		// aabb: 要插入的 AABB
		// 返回值: 子节点的索引，如果不适合任何子节点则返回 -1
		int getIndex(const AABB& aabb);

		// 存储当前节点包含的所有 AABB
		std::vector<unsigned> m_objects;
		// 当前节点所代表的 AABB 区域
		AABB m_bounds;
		// 四个子节点的指针数组
		QuadTreeNode* m_nodes[4] = { nullptr };
		// 当前节点所在的层级
		int m_level;
		// 四叉树允许的最大层级
		int m_maxLevel;
		// 每个节点允许存储的最大 AABB 数量
		int m_maxObjects;
	};

	// 四叉树类，用于管理四叉树结构
	class QuadTree {
	public:
		// 构造函数，初始化四叉树
		// bounds: 四叉树根节点所代表的 AABB 区域
		// maxLevel: 四叉树允许的最大层级，默认为 5
		// maxObjects: 每个节点允许存储的最大 AABB 数量，默认为 10
		QuadTree(const AABB& bounds, int maxLevel = 5, int maxObjects = 10)
			: m_root(bounds, 0, maxLevel, maxObjects) {
		}
		// 根据一组AABB来得到AABB所包含的空间范围
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

		// 向四叉树中插入一个 AABB
		// aabb: 要插入的 AABB
		void insert(const AABB& aabb) {
			m_root.insert(aabb);
		}

		// 从四叉树中检索与指定 AABB 可能相交的所有 AABB 的索引
		// aabb: 用于检索的 AABB
		// 返回值: 包含可能相交的 AABB 索引的向量
		std::vector<unsigned int> retrieve(const AABB& aabb) {
			return m_root.retrieve(aabb);
		}

	private:
		// 四叉树的根节点
		QuadTreeNode m_root;
	};

	// 网格划分类，用于将空间划分为网格单元
	class Grid {
	public:
		// 构造函数，初始化网格
		// bounds: 网格所覆盖的 AABB 区域
		// cellSize: 每个网格单元的大小
		Grid(const AABB& bounds, int cellSize)
			: m_bounds(bounds), m_cellSize(cellSize) {
			int width = static_cast<int>(bounds.getWidth() / cellSize);
			int height = static_cast<int>(bounds.getHeight() / cellSize);
			// 初始化网格单元数组
			m_cells.resize(width * height);
		}

		// 向网格中插入一个 AABB
		// aabb: 要插入的 AABB
		void insert(const AABB& aabb) {
			// 计算 AABB 覆盖的起始和结束网格单元索引
			int startX = static_cast<int>((aabb.getTopLeft().x() - m_bounds.getTopLeft().x()) / m_cellSize);
			int startY = static_cast<int>((aabb.getTopLeft().y() - m_bounds.getTopLeft().y()) / m_cellSize);
			int endX = static_cast<int>((aabb.getBottomRight().x() - m_bounds.getTopLeft().x()) / m_cellSize);
			int endY = static_cast<int>((aabb.getBottomRight().y() - m_bounds.getTopLeft().y()) / m_cellSize);

			// 将 AABB 插入到其覆盖的所有网格单元中
			for (int y = startY; y <= endY; ++y) {
				for (int x = startX; x <= endX; ++x) {
					int index = y * static_cast<int>(m_bounds.getWidth() / m_cellSize) + x;
					if (index >= 0 && index < m_cells.size()) {
						m_cells[index].push_back(aabb);
					}
				}
			}
		}

		// 从网格中检索与指定 AABB 可能相交的所有 AABB
		// aabb: 用于检索的 AABB
		// 返回值: 包含可能相交的 AABB 的向量
		std::vector<AABB> retrieve(const AABB& aabb) {
			std::vector<AABB> result;
			// 计算 AABB 覆盖的起始和结束网格单元索引
			int startX = static_cast<int>((aabb.getTopLeft().x() - m_bounds.getTopLeft().x()) / m_cellSize);
			int startY = static_cast<int>((aabb.getTopLeft().y() - m_bounds.getTopLeft().y()) / m_cellSize);
			int endX = static_cast<int>((aabb.getBottomRight().x() - m_bounds.getTopLeft().x()) / m_cellSize);
			int endY = static_cast<int>((aabb.getBottomRight().y() - m_bounds.getTopLeft().y()) / m_cellSize);

			// 从 AABB 覆盖的所有网格单元中收集可能相交的 AABB
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
		// 网格所覆盖的 AABB 区域
		AABB m_bounds;
		// 每个网格单元的大小
		int m_cellSize;
		// 存储所有网格单元的二维数组
		std::vector<std::vector<AABB>> m_cells;
	};
}