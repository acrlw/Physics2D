#ifndef PHYSICS2D_BROADPHASE_DBVH_H
#define PHYSICS2D_BROADPHASE_DBVH_H
#include "aabb.h"

namespace Physics2D
{
	/// <summary>
	/// Dynamic Bounding Volume Hierarchy
	///	This is implemented by traditional binary search tree
	/// </summary>
	class DBVH
	{
		public:
			struct Node
			{
				Node(Body* _body, const AABB& _aabb) : body(_body), aabb(_aabb){}
				Node(const AABB& _aabb) : aabb(_aabb) {}
				Node* parent = nullptr;
				Node* left = nullptr;
				Node* right = nullptr;
				Body* body = nullptr;
				AABB aabb;
				void separate(Node* node);
				void swap(Node* source, Node* target);
				bool isLeaf()const;
				bool isBranch()const;
				bool isRoot()const;
				void clear();
			};
		
			DBVH();
			~DBVH();
			void insert(Body* body);
			void update(Body* body);
			Node* extract(Body* body);
			void erase(Body* body);
			void cleanUp(Node* node);
			Node* root()const;
			Container::Vector<Body*> raycast(const Vector2& start, const Vector2& direction);
			Container::Vector<std::pair<Body*, Body*>> generate();
			Container::Map<Body*, Node*>& leaves();
			void query(const AABB& sourceAABB, Container::Vector<Node*>& nodes, Body* skipBody = nullptr)const;
			static void queryNodes(Node* node, const AABB& sourceAABB, Container::Vector<Node*>& nodes, Body* skipBody = nullptr);
		private:
			void raycast(Container::Vector<Body*>& result, Node* node, const Vector2& start, const Vector2& direction);
			void insert(Node* node);
			real deltaCost(Node* node, const AABB& aabb)const;
			void totalCost(Node* node, const AABB& aabb, real& cost)const;
			Node* merge(Node* node, const AABB& aabb, Body* body);
			void merge(Node* target, Node* source);
			void update(Node* parent);
			void balance(Node* node);
			void generate(Node* node, Container::Vector<std::pair<Body*, Body*>>& pairs);
			void generate(Node* left, Node* right, Container::Vector<std::pair<Body*, Body*>>& pairs);
			int height(Node* node);

			Container::Map<Body*, Node*> m_leaves;
			Node* m_root = nullptr;
			real m_profile = 0;
			real m_leafFactor = 0.5;
	};
}
#endif