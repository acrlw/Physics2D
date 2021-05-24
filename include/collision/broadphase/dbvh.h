#ifndef PHYSICS2D_BROADPHASE_DBVH_H
#define PHYSICS2D_BROADPHASE_DBVH_H
#include "aabb.h"

namespace Physics2D
{
	
	class DBVH
	{
		public:
			struct Node
			{
				Node(const Pair& pair) : pair(pair){}
				Node(const AABB& aabb) : pair(aabb){}
				Node* parent = nullptr;
				Node* left = nullptr;
				Node* right = nullptr;
				Pair pair;
				void separate(Node* node);
				void swap(Node* source, Node* target);
				bool isLeaf()const;
				bool isBranch()const;
				bool isRoot()const;
			};
		
			DBVH();
			~DBVH();
			void insert(Body* body);
			void update(Body* body);
			Node* extract(Body* body);
			void erase(Body* body);
			Node* raycast(const Vector2& start, const Vector2& direction);
			Node* root()const;
			std::vector<std::pair<Body*, Body*>> generatePairs();

			std::map<Body*, Node*>& leaves();
		private:
			void insert(Node* node);
			void cleanUp(Node* node);
			real deltaCost(Node* node, const AABB& aabb)const;
			void totalCost(Node* node, const AABB& aabb, real& cost)const;
			Node* merge(Node* node, const Pair& pair);
			void merge(Node* target, Node* source);
			void update(Node* parent);
			void balance(Node* node);
			void generate(Node* node, std::vector<std::pair<Body*, Body*>>& pairs);
			void generate(Node* left, Node* right, std::vector<std::pair<Body*, Body*>>& pairs);
			int height(Node* node);

			std::map<Body*, Node*> m_leaves;
			Node* m_root = nullptr;
			real m_profile = 0;
			real m_leafFactor = 0.1;
	};
}
#endif