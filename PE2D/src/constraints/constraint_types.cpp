#include "constraints/constraint_types.hpp"

namespace PE2D {
	std::vector<std::shared_ptr<Constraint>> Constraint::ID_Map;
	void Constraint::removeFromMap(unsigned int id) {
		// Remove the object from the ID map
		//减少对象的引用计数
		ID_Map[id] = nullptr;
	}
	bool Constraint::isValidID(unsigned int id) {
		//判断ID代表的物体是否有效，物体是否存在
		if (id < ID_Map.size()) {
			if (ID_Map[id] == nullptr)
				return false;
			else
				return true;
		}
		else {
			return false;
		}
	}
	unsigned int Constraint::getNextID() const {
		unsigned int nextID = 1;
		while (nextID < ID_Map.size() && ID_Map[nextID] != nullptr) {
			++nextID;
		}
		return nextID;
	}
}