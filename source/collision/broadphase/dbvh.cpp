#include "include/collision/broadphase/dbvh.h"

namespace Physics2D
{
	DBVH::DBVH()
	{
	}

	DBVH::~DBVH()
	{
		cleanUp(m_root);
	}

	DBVH::Node* DBVH::root() const
	{
		return m_root;
	}

	std::vector<AABBPair> DBVH::generatePairs()
	{
		m_profile = 0;
		std::vector<AABBPair> pairs;
		generate(m_root, pairs);
		return pairs;
	}

	void DBVH::insert(const AABB& aabb)
	{
		auto getCost = [&](Node*& node, const AABB& aabb)
		{
			if (node->isLeaf())
				return std::make_tuple(node, AABB::unite(node->value, aabb).surfaceArea());

			std::vector<Node*> leaves;
			traverseToLeaf(node, leaves);
			Node* lowCost = nullptr;
			real costValue = Constant::Max;
			for (Node* node : leaves)
			{
				real cost = 0;
				totalCost(node, aabb, cost);
				if (costValue > cost)
				{
					costValue = cost;
					lowCost = node;
				}
			}

			return std::make_tuple(lowCost, costValue);
		};

		if (m_root == nullptr)
		{
			m_root = new Node(aabb);
			return;
		}

		if (m_root->isLeaf() && m_root->isRoot())
		{
			merge(m_root, aabb);
			update(m_root);
			return;
		}

		if (m_root->isRoot())
		{
			auto [leftTarget, leftLowestCost] = getCost(m_root->left, aabb);
			auto [rightTarget, rightLowestCost] = getCost(m_root->right, aabb);
			if (leftLowestCost < rightLowestCost)
			{
				merge(leftTarget, aabb);
				//update(leftTarget);
				balance(m_root);
				update(leftTarget);
			}
			else
			{
				merge(rightTarget, aabb);
				//update(rightTarget);
				balance(m_root);
				update(rightTarget);
			}
		}

		//wrap
	}

	void DBVH::merge(Node* node, const AABB& aabb)
	{
		Node* newNode = new Node(aabb);
		Node* copy = new Node(node->value);
		node->value = AABB::unite(aabb, node->value);
		node->left = copy;
		node->right = newNode;
		copy->parent = node;
		newNode->parent = node;
	}

	void DBVH::update(Node* parent)
	{
		if (parent == nullptr)
			return;

		if (parent->left != nullptr && parent->right != nullptr)
			parent->value = AABB::unite(parent->left->value, parent->right->value);

		update(parent->parent);

		if (parent->parent == nullptr && parent->left != nullptr && parent->right != nullptr)
			m_root = parent;
	}

	void DBVH::balance(Node* node)
	{
		if (node == nullptr)
			return;

		const int leftHeight = height(node->left);
		const int rightHeight = height(node->right);
		if (abs(leftHeight - rightHeight) <= 1)
			return;

		auto LL = [&](Node* node)
		{
			if (node->parent == m_root)
			{
				Node* parent = node->parent;
				Node* right = node->right;
				parent->left = right;
				node->parent = nullptr;

				node->right = parent;
				right->parent = parent;
				parent->left = right;

				parent->parent = node;
				m_root = node;
				return;
			}
			Node* parent = node->parent;
			Node* right = node->right;
			Node* grandparent = parent->parent;

			node->parent = grandparent;
			if (parent == grandparent->left)
				grandparent->left = node;
			else
				grandparent->right = node;

			node->right = parent;
			parent->parent = node;

			parent->left = right;
			right->parent = parent;
		};
		auto RR = [&](Node* node)
		{
			if (node->parent == m_root)
			{
				Node* parent = node->parent;
				Node* left = node->left;
				parent->right = left;
				node->parent = nullptr;

				node->left = parent;
				left->parent = parent;

				parent->right = left;
				parent->parent = node;

				m_root = node;
				return;
			}
			Node* parent = node->parent;
			Node* left = node->left;
			Node* grandparent = parent->parent;

			node->parent = grandparent;
			if (parent == grandparent->right)
				grandparent->right = node;
			else
				grandparent->left = node;

			node->left = parent;
			parent->parent = node;

			parent->right = left;
			left->parent = parent;
		};
		if (leftHeight > rightHeight) //left unbalance
		{
			const int leftLeftHeight = height(node->left->left);
			const int leftRightHeight = height(node->left->right);
			if (leftLeftHeight < leftRightHeight) //LR case
				RR(node->left->right);
			LL(node->left);
		}
		else //right unbalance
		{
			const int rightRightHeight = height(node->right->right);
			const int rightLeftHeight = height(node->right->left);
			if (rightRightHeight < rightLeftHeight) //RL case
				LL(node->right->left);
			RR(node->right);
		}
		balance(node->left);
		balance(node->right);
	}

	//check if children collide with each other
	void DBVH::generate(Node* node, std::vector<AABBPair>& pairs)
	{
		if (node->isLeaf())
			return;

		bool result = AABB::collide(node->left->value, node->right->value);
		m_profile++;
		if (result)
			generate(node->left, node->right, pairs);
		
		generate(node->left, pairs);
		generate(node->right, pairs);
	}

	void DBVH::generate(Node* left, Node* right, std::vector<AABBPair>& pairs)
	{
		if (left == nullptr || right == nullptr)
			return;

		bool result = left->value.collide(right->value);
		m_profile++;
		
		if (!result)
			return;

		if (left->isLeaf() && right->isLeaf())
		{
			m_profile++;
			AABBPair pair = {left->value, right->value};
			pairs.emplace_back(pair);
		}
		if (left->isLeaf() && right->isBranch())
		{
			generate(left, right->left, pairs);
			generate(left, right->right, pairs);
		}
		if (right->isLeaf() && left->isBranch())
		{
			generate(right, left->left, pairs);
			generate(right, left->right, pairs);
		}
		if (left->isBranch() && right->isBranch())
		{
			generate(left->left, right, pairs);
			generate(left->right, right, pairs);
		}
	}

	int DBVH::height(Node* node)
	{
		return node == nullptr ? 0 : Math::max(height(node->left), height(node->right)) + 1;
	}

	void DBVH::cleanUp(Node* node)
	{
		if (node == nullptr)
			return;

		cleanUp(node->left);
		cleanUp(node->right);

		delete node;
		node = nullptr;
	}

	void DBVH::traverseToLeaf(Node*& node, std::vector<Node*>& leaves)
	{
		if (node == nullptr)
			return;
		if (node->isLeaf())
		{
			leaves.emplace_back(node);
			return;
		}
		traverseToLeaf(node->left, leaves);
		traverseToLeaf(node->right, leaves);
	}

	real DBVH::deltaCost(Node* node, const AABB& aabb)
	{
		if (node->isLeaf())
			return AABB::unite(node->value, aabb).surfaceArea();

		return AABB::unite(node->value, aabb).surfaceArea() - node->value.surfaceArea();
	}

	void DBVH::totalCost(Node* node, const AABB& aabb, real& cost)
	{
		if (node == nullptr)
			return;

		cost += deltaCost(node, aabb);
		totalCost(node->parent, aabb, cost);
	}

	bool DBVH::Node::isLeaf() const
	{
		return left == nullptr && right == nullptr;
	}

	bool DBVH::Node::isBranch() const
	{
		return left != nullptr && right != nullptr && parent != nullptr;
	}

	bool DBVH::Node::isRoot() const
	{
		return parent == nullptr;
	}
}
