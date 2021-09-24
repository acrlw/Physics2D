#ifndef PHYSICS2D_BROADPHASE_DBVT_H
#define PHYSICS2D_BROADPHASE_DBVT_H

#include "aabb.h"

namespace Physics2D
{
	class DBVT
	{
	public:
		struct Node
		{
			Body* body = nullptr;
			AABB aabb;
			int parentIndex = -1;
			int leftIndex = -1;
			int rightIndex = -1;
			bool isLeaf()const;
			bool isBranch()const;
			bool isRoot()const;
			bool isEmpty()const;
			void clear();
			
		};
		DBVT();
		std::vector<Body*> raycast(const Vector2& point, const Vector2& direction);
		std::vector<std::pair<Body*, Body*>> generatePairs();
		void insert(Body* body);
		void remove(Body* body);

		std::vector<Node>& tree();
		int rootIndex()const;
	private:
		void extract(int targetIndex);
		void merge(int targetIndex);
		void balance(int targetIndex);
		void updateNodeIndex(int targetIndex);
		int allocateNode();
		int height(int targetIndex);

		int m_rootIndex = -1;
		std::vector<Node> m_tree;
		std::vector<int> m_emptyList;
		std::map<Body*, int> m_bodyTable;
	};
}

#endif
