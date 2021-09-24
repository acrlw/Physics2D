#include "include/collision/broadphase/dbvh.h"

#include "include/dynamics/body.h"

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
	std::map<Body*, DBVH::Node*>& DBVH::leaves()
	{
		return m_leaves;
	}

	void DBVH::query(const AABB& sourceAABB, std::vector<Node*>& nodes, Body* skipBody)const
	{
		queryNodes(m_root, sourceAABB, nodes, skipBody);
	}

	void DBVH::queryNodes(Node* node, const AABB& aabb, std::vector<Node*>& nodes, Body* skipBody)
	{
		if (node == nullptr || !aabb.collide(node->pair.aabb))
			return;

		//skip query
		if (skipBody != nullptr)
			if (node->pair.body == skipBody)
				return;

		if (node->isBranch() || node->isRoot())
		{
			queryNodes(node->left, aabb, nodes, skipBody);
			queryNodes(node->right, aabb, nodes, skipBody);
			return;
		}

		if (node->isLeaf())
			nodes.emplace_back(node);
	}

	void DBVH::raycast(std::vector<Body*>& result, Node* node, const Vector2& start, const Vector2& direction)
	{
		if (node == nullptr)
			return;
		//for(auto iter = m_leaves.begin(); iter != m_leaves.end(); ++iter)
		//	if(iter->second->pair.aabb.raycast(start, direction))
		//		result.emplace_back(iter->first);
		
		//if(node->pair.aabb.raycast(start, direction))
		//{
		//	if (node->isLeaf())
		//		result.emplace_back(node->pair.body);
		//	else
		//	{
		//		raycast(result, node->left, start, direction);
		//		raycast(result, node->right, start, direction);
		//	}
		//}

	}

	void DBVH::insert(Node* node)
	{
		if (node == nullptr)
			return;
		AABB aabb = AABB::fromBody(node->pair.body);
		aabb.expand(m_leafFactor);
		node->pair.aabb = aabb;

		auto getCost = [&](Node* target, const AABB& aabb)
		{
			Node* lowCostNode = nullptr;
			real costValue = Constant::Max;
			for (auto const& [key, val] : m_leaves)
			{
				assert(val != nullptr);
				if(val == target)
					continue;
				real cost = 0;
				totalCost(val, aabb, cost);
				if (costValue > cost)
				{
					costValue = cost;
					lowCostNode = val;
				}
			}

			return lowCostNode;
		};

		if (m_root == nullptr)
		{
			m_root = node;
			return;
		}

		if (m_root->isLeaf() && m_root->isRoot())
		{
			merge(m_root, node);
			update(m_root);
			return;
		}

		if (m_root->isRoot())
		{
			auto target = getCost(node, aabb);
			merge(target, node);
			balance(m_root);

			for (auto const& [key, val] : m_leaves)
			{
				if (val == node)
					continue;
				update(val);
			}
		}
	}

	void DBVH::insert(Body* body)
	{
		if(m_leaves.contains(body))
			return;
		
		AABB aabb = AABB::fromBody(body);
		aabb.expand(m_leafFactor);
		Pair pair(aabb, body);
		
		auto getCost = [&](const AABB& aabb)
		{
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

			return lowCostNode;
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
			auto target = getCost(pair.aabb);
			merge(target, pair);
			
			balance(m_root);

			update(target);
			//for (auto const& [key, val] : m_leaves)
			//	update(val);
		}
	}
	void DBVH::update(Body* body)
	{
		assert(body != nullptr);

		auto iter = m_leaves.find(body);
		if (iter == m_leaves.end())
			return;
		
		
		AABB thin = AABB::fromBody(body);
		thin.expand(0.1);
		if(!thin.isSubset(iter->second->pair.aabb))
		{
			Node* node = extract(body);
			insert(node);
		}

	}
	DBVH::Node* DBVH::extract(Body* body)
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
			branch->pair.aabb.clear();
			delete branch;
			return child;
		};

		//separate
		auto iter = m_leaves.find(body);

		if (iter == m_leaves.end())
			return nullptr;

		

		Node* source = iter->second;
		Node* parent = source->parent;
		parent->separate(source);

		if (iter->second->isRoot() && iter->second->isLeaf() && m_leaves.size() < 2)
		{
			m_root = nullptr;
			return source;
		}
		
		Node* child = moveup(parent);
		
		balance(child->parent);
		update(child->parent);
		return source;
	}
	void DBVH::erase(Body* body)
	{
		Node* target = extract(body);
		
		if (target == nullptr)
			return;

		target->pair.body = nullptr;
		target->pair.aabb.clear();
		delete target;
		m_leaves.erase(body);
	}
	std::vector<Body*> DBVH::raycast(const Vector2& start, const Vector2& direction)
	{
		std::vector<Body*> result;
		raycast(result, m_root, start, direction);
		return result;
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
		node->pair.aabb = AABB::unite(pair.aabb, node->pair.aabb);
		//node->pair.aabb.expand(0.5);
		node->left = copy;
		node->right = newNode;
		copy->parent = node;
		newNode->parent = node;

		return newNode;
	}

	void DBVH::merge(Node* target, Node* source)
	{
		assert(target != nullptr && source != nullptr);
		assert(source->isLeaf());
		
		Node* copy = new Node(target->pair);
		if (target->isLeaf())
			m_leaves[target->pair.body] = copy;
		target->pair.body = nullptr;
		target->pair.aabb = AABB::unite(copy->pair.aabb, source->pair.aabb);
		target->left = copy;
		target->right = source;
		copy->parent = target;
		source->parent = target;
		
	}

	void DBVH::update(Node* parent)
	{
		if (parent == nullptr)
			return;

		if (parent->isBranch() || parent->isRoot())
			parent->pair.aabb = AABB::unite(parent->left->pair.aabb, parent->right->pair.aabb);

		
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
			else
				LL(node->left->left);
			LL(node->left);
		}
		else //right unbalance
		{
			const int rightRightHeight = height(node->right->right);
			const int rightLeftHeight = height(node->right->left);
			if (rightRightHeight < rightLeftHeight) //RL case
				LL(node->right->left);
			else
				RR(node->right->right);
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

		bool result = AABB::collide(node->left->pair.aabb, node->right->pair.aabb);
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

		bool result = left->pair.aabb.collide(right->pair.aabb) || left->pair.aabb.isSubset(right->pair.aabb);
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
			return AABB::unite(node->pair.aabb, aabb).surfaceArea();

		return AABB::unite(node->pair.aabb, aabb).surfaceArea() - node->pair.aabb.surfaceArea();
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
		if (node == nullptr)
			return;

		if (node->isLeaf() && node->isRoot())
			return;

		if (node == left)
		{
			left = nullptr;
			node->parent = nullptr;
			pair.aabb = right->pair.aabb;
		}
		else if(node == right)
		{
			right = nullptr;
			node->parent = nullptr;
			pair.aabb = left->pair.aabb;
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
