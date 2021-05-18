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

	std::vector<std::pair<Body*, Body*>> DBVH::generatePairs()
	{
		m_profile = 0;
		std::vector<std::pair<Body*, Body*>> pairs;
		generate(m_root, pairs);
		return pairs;
	}
	void DBVH::insert(Body* body)
	{
		AABB aabb = AABB::fromBody(body);
		aabb.width += m_leafFactor;
		aabb.height += m_leafFactor;
		Pair pair(aabb, body);
		
		auto getCost = [&](Node*& node, const AABB& aabb)
		{
			if (node->isLeaf())
				return std::make_tuple(node, AABB::unite(node->pair.value, aabb).surfaceArea());
			
			Node* lowCostNode = nullptr;
			real costValue = Constant::Max;
			for (auto const& [key, val] : m_leaves)
			{
				assert(val != nullptr);
				real cost = 0;
				totalCost(val, aabb, cost);
				if(costValue > cost)
				{
					costValue = cost;
					lowCostNode = val;
				}
			}

			return std::make_tuple(lowCostNode, costValue);
		};

		if (m_root == nullptr)
		{
			m_root = new Node(pair);
			m_leaves.insert({ body, m_root });
			return;
		}

		if (m_root->isLeaf() && m_root->isRoot())
		{
			merge(m_root, pair);
			update(m_root);
			return;
		}

		if (m_root->isRoot())
		{
			auto [leftTarget, leftLowestCost] = getCost(m_root->left, pair.value);
			auto [rightTarget, rightLowestCost] = getCost(m_root->right, pair.value);
			Node* target1 = nullptr;
			Node* target2 = nullptr;
			if (leftLowestCost < rightLowestCost)
			{
				target1 = merge(leftTarget, pair);
				target2 = leftTarget;
			}
			else
			{
				target1 = merge(rightTarget, pair);
				target2 = rightTarget;
			}
			balance(m_root);
			//update(target2);
			//update(target1);

			for (auto const& [key, val] : m_leaves)
			{
				update(val);
			}
		}
	}
	void DBVH::update(Body* body)
	{
		AABB thin = AABB::fromBody(body, 1.2);
		if(!thin.isSubset(m_leaves[body]->pair.value))
		{
			remove(body);
			insert(body);
		}
	}
	void DBVH::remove(Body* body)
	{
		auto moveup = [&](Node* branch)
		{
			Node* parent = branch->parent;
			Node* child = branch->left != nullptr ? branch->left : branch->right;
			
			if (branch->isRoot())
			{
				child->parent = nullptr;
				m_root = child;
				delete branch;
				return child;
			}
			
			parent->swap(branch, child);
			branch->parent = nullptr;
			branch->pair.value.clear();
			delete branch;
			return child;
		};
		
		//separate
		auto iter = m_leaves.find(body);
		
		if (iter == m_leaves.end())
			return;
		
		Node* source = iter->second;
		Node* parent = source->parent;
		parent->separate(source);
		source->pair.body = nullptr;
		source->pair.value.clear();
		delete source;

		Node * child = moveup(parent);
		
		//erase
		m_leaves.erase(body);
		balance(child->parent);
	}
	DBVH::Node* DBVH::merge(Node* node, const Pair& pair)
	{
		assert(node != nullptr);
		
		Node* newNode = new Node(pair);
		Node* copy = new Node(node->pair);

		m_leaves.insert({ newNode->pair.body, newNode });
		if (node->isLeaf())
			m_leaves[node->pair.body] = copy;

		
		node->pair.body = nullptr;
		node->pair.value = AABB::unite(pair.value, node->pair.value);
		node->left = copy;
		node->right = newNode;
		copy->parent = node;
		newNode->parent = node;

		return newNode;
	}

	void DBVH::update(Node* parent)
	{
		if (parent == nullptr)
			return;

		if (parent->isBranch() || parent->isRoot())
			parent->pair.value = AABB::unite(parent->left->pair.value, parent->right->pair.value);

		
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
			if (node == nullptr || node->isRoot())
				return;
			
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
			if (node == nullptr || node->isRoot())
				return;
			
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
		balance(node->parent);
	}

	//check if children collide with each other
	void DBVH::generate(Node* node, std::vector<std::pair<Body*, Body*>>& pairs)
	{
		if (node == nullptr || node->isLeaf())
			return;

		bool result = AABB::collide(node->left->pair.value, node->right->pair.value);
		m_profile++;
		if (result)
			generate(node->left, node->right, pairs);
		
		generate(node->left, pairs);
		generate(node->right, pairs);
	}

	void DBVH::generate(Node* left, Node* right, std::vector<std::pair<Body*, Body*>>& pairs)
	{
		if (left == nullptr || right == nullptr)
			return;

		bool result = left->pair.value.collide(right->pair.value) || left->pair.value.isSubset(right->pair.value);
		m_profile++;
		
		if (!result)
			return;

		if (left->isLeaf() && right->isLeaf())
		{
			m_profile++;
			std::pair<Body*, Body*> pair = {left->pair.body, right->pair.body};
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
	real DBVH::deltaCost(Node* node, const AABB& aabb)const
	{
		if (node == nullptr)
			return 0;
		
		if (node->isLeaf())
			return AABB::unite(node->pair.value, aabb).surfaceArea();

		return AABB::unite(node->pair.value, aabb).surfaceArea() - node->pair.value.surfaceArea();
	}

	void DBVH::totalCost(Node* node, const AABB& aabb, real& cost)const
	{
		if (node == nullptr)
			return;

		const real temp = deltaCost(node, aabb);
		cost += temp;
		totalCost(node->parent, aabb, cost);
	}

	void DBVH::Node::separate(Node* node)
	{
		if (node == nullptr || isLeaf())
			return;

		if (node == left)
		{
			left = nullptr;
			node->parent = nullptr;
			pair.value = right->pair.value;
		}
		else if(node == right)
		{
			right = nullptr;
			node->parent = nullptr;
			pair.value = left->pair.value;
		}
	}

	void DBVH::Node::swap(Node* source, Node* target)
	{
		if(source == left)
		{
			separate(left);
			target->parent = this;
			left = target;
		}
		else if (source == right)
		{
			separate(right);
			target->parent = this;
			right = target;
		}
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
