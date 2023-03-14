#ifndef PHYSICS2D_BROADPHASE_DBVT_H
#define PHYSICS2D_BROADPHASE_DBVT_H

#include "aabb.h"

namespace Physics2D
{
	/// <summary>
	/// Dynamic Bounding Volume Tree
	///	This is implemented by dynamic array-arranged.
	/// </summary>
	class Tree
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
		Tree();
		Container::Vector<Body*> query(Body* body);
		Container::Vector<Body*> query(const AABB& aabb);
		Container::Vector<Body*> raycast(const Vector2& point, const Vector2& direction);
		Container::Vector<std::pair<Body*, Body*>> generate();
		void insert(Body* body);
		void remove(Body* body);
		void clearAll();
		void update(Body* body);
		const Container::Vector<Node>& tree();
		int rootIndex()const;
	private:
		void queryNodes(int nodeIndex, const AABB& aabb, Container::Vector<Body*>& result);
		void traverseLowestCost(int nodeIndex, int boxIndex, real& cost, int& finalIndex);
		void raycast(Container::Vector<Body*>& result, int nodeIndex, const Vector2& p, const Vector2& d);
		void generate(int nodeIndex, Container::Vector<std::pair<Body*, Body*>>& pairs);
		void generate(int leftIndex, int rightIndex, Container::Vector<std::pair<Body*, Body*>>& pairs);
		void extract(int targetIndex);
		int merge(int nodeIndex, int leafIndex);
		void ll(int nodeIndex);
		void rr(int nodeIndex);
		void balance(int targetIndex);
		void separate(int sourceIndex, int parentIndex);
		void join(int nodeIndex, int boxIndex);
		void remove(int targetIndex);
		void elevate(int targetIndex);
		void upgrade(int nodeIndex);
		int calculateLowestCostNode(int nodeIndex);
		real totalCost(int nodeIndex, int leafIndex);
		real deltaCost(int nodeIndex, int boxIndex);
		size_t allocateNode();
		int height(int targetIndex);

		real m_fatExpansionFactor = 0.5f;
		int m_rootIndex = -1;
		Container::Vector<Node> m_tree;
		Container::Vector<int> m_emptyList;
		Container::Map<Body*, int> m_bodyTable;
	};

	
}

#endif
