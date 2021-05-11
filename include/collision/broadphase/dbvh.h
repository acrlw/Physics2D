#ifndef PHYSICS2D_BROADPHASE_DBVH_H
#define PHYSICS2D_BROADPHASE_DBVH_H
#include "include/collision/broadphase/aabb.h"

namespace Physics2D
{
	struct BodyPair
	{
		Body* bodyA;
		Body* bodyB;
	};
	
	class DBVH
	{
		public:
			struct Pair
			{
				Pair(const AABB& aabb, Body* source = nullptr): body(source), value(aabb){}
				Body* body = nullptr;
				AABB value;
			};
			struct Node
			{
				Node(const Pair& pair) : pair(pair){}
				Node(const AABB& aabb) : pair(aabb){}
				Node* parent = nullptr;
				Node* left = nullptr;
				Node* right = nullptr;
				Pair pair;
				bool isLeaf()const;
				bool isBranch()const;
				bool isRoot()const;
			};
		
			DBVH();
			~DBVH();
			void insert(Body* body);
			Node* root()const;
			std::vector<BodyPair> generatePairs();
		private:
			void insert(const AABB& aabb);
			void cleanUp(Node* node);
			void traverseToLeaf(Node*& node, std::vector<Node*>& leaves);
			real deltaCost(Node* node, const AABB& aabb);
			void totalCost(Node* node, const AABB& aabb, real& cost);
			void merge(Node* node, const Pair& pair);
			void update(Node* parent);
			void balance(Node* node);
			void generate(Node* node, std::vector<BodyPair>& pairs);
			void generate(Node* left, Node* right, std::vector<BodyPair>& pairs);
			int height(Node* node);
			Node* m_root = nullptr;
			real m_profile = 0;
			real m_leafFactor = 1.5;
	};
}
#endif