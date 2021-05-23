#ifndef PHYSICS2D_BROADPHASE_TREE_H
#define PHYSICS2D_BROADPHASE_TREE_H
#include "aabb.h"
namespace Physics2D
{
	/// <summary>
	/// Dynamic Bounding Volume Tree
	/// Using vector to implement tree.
	/// </summary>
	class Tree
	{
	public:
		struct Node
		{
			Node(){};
			Node(Body* b, int i);
			AABB aabb;
			Body* body = nullptr;
			int index = -1;
			int left = -1;
			int right = -1;
			int parent = -1;
			bool isEmpty()const;
			void clear();
		};
		void insert(Body* body);
		void erase(Body* body);
		void extract(Body* body);
		void update(Body* body);
	private:
		void balance(int targetIndex);
		void update(int targetIndex);
		void expand(int levels = 1);
		int allocateNode();
		bool isLeaf(int targetIndex)const;
		bool isBranch(int targetIndex)const;
		bool isRoot(int targetIndex)const;
		Node separate(int targetIndex, int sourceIndex);
		void merge(int targetIndex, const Node& node);
		real deltaCost(int nodeIndex, const AABB& aabb)const;
		real totalCost(int nodeIndex, const AABB& aabb)const;
		
		std::vector<Node> m_tree;
		std::map<Body*, int> m_leaves;
		real m_leafFactor = 0.5;
	};
	
}
#endif