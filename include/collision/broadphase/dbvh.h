#ifndef PHYSICS2D_BROADPHASE_DBVH_H
#define PHYSICS2D_BROADPHASE_DBVH_H
#include "include/collision/broadphase/aabb.h"

namespace Physics2D
{
	struct AABBPair
	{
		AABB aabb1;
		AABB aabb2;
	};
	class DBVH
	{
		public:

			struct Node
			{
				Node(const AABB& aabb) : value(aabb){}
				Node* parent = nullptr;
				Node* left = nullptr;
				Node* right = nullptr;
				AABB value;
				bool isLeaf()const;
				bool isBranch()const;
				bool isRoot()const;
			};
		
			DBVH();
			~DBVH();
			void insert(const AABB& aabb);
			Node* root()const;
			std::vector<AABBPair> generatePairs();
		private:
			void cleanUp(Node* node);
			void traverseToLeaf(Node*& node, std::vector<Node*>& leaves);
			real deltaCost(Node* node, const AABB& aabb);
			void totalCost(Node* node, const AABB& aabb, real& cost);
			void merge(Node* node, const AABB& aabb);
			void update(Node* parent);
			void balance(Node* node);
			void generate(Node* node, std::vector<AABBPair>& pairs);
			void generate(Node* left, Node* right, std::vector<AABBPair>& pairs);
			int height(Node* node);
			Node* m_root = nullptr;
			real m_profile = 0;
	};
}
#endif