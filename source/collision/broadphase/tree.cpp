#include "include/collision/broadphase/tree.h"
namespace Physics2D
{
	Tree::Node::Node(int i):index(i)
	{
		left = 2 * i + 1;
		right = 2 * i + 2;
		parent = (i - 1) / 2;
	}
	bool Tree::Node::isEmpty() const
	{
		return pair.aabb.isEmpty();
	}
	void Tree::Node::clear()
	{
		pair.clear();
	}
	void Tree::insert(Body* body)
	{
		assert(body != nullptr);
		
		if(std::find_if(m_tree.begin(), m_tree.end(), [body](const Node& node)
			{
				return node.pair.body == body;
			}) != m_tree.end())
			return;

		Pair pair;
		AABB aabb = AABB::fromBody(body);
		aabb.expand(m_leafFactor);
		pair.body = body;
		pair.aabb = aabb;
		
		
		auto getCost = [&](const AABB& aabb)
		{
			int targetIndex = -1;
			real minimumCost = Constant::Max;
			for(auto&[body, leafIndex]: m_leaves)
			{
				real cost = totalCost(leafIndex, aabb);
				if(minimumCost > cost)
				{
					targetIndex = leafIndex;
					minimumCost = cost;
				}
			}
			return targetIndex;
		};

		
		if(m_leaves.empty())
		{
			expand(1);
			m_tree[0].pair = pair;
			m_tree[0].parent = -1;
			m_leaves[body] = 0;
			return;
		}

		if(m_leaves.size() == 1)
		{
			expand(1);
			merge(0, pair);
			return;
		}
		//get target leaf node
		const int targetIndex = getCost(pair.aabb);
		if (2 * targetIndex + 2 >= m_tree.size())
			expand(1);

		merge(targetIndex, pair);
		//m_leaves[body] = m_tree[targetIndex].right;
		
		for (auto& [body, index] : m_leaves)
			update(index);
		
	}

	void Tree::erase(Body* body)
	{
		if (body == nullptr)
			return;

		Node& target = m_tree[m_leaves[body]];
		Node& parent = m_tree[target.parent];
		Node& child = target.index == parent.left ? m_tree[parent.right] : m_tree[parent.left];
		
		m_leaves.erase(target.pair.body);
		target.clear();
		parent.pair = child.pair;
		child.clear();
		m_leaves[parent.pair.body] = parent.index;
	}
	void Tree::update(int targetIndex)
	{
		if (targetIndex > m_tree.size())
			return;

		Node& target = m_tree[targetIndex];
		if (target.right >= m_tree.size())
			return;
		Node& left = m_tree[target.left];
		Node& right = m_tree[target.right];
		if (!left.isEmpty() && !right.isEmpty())
			target.pair.aabb = AABB::unite(left.pair.aabb, right.pair.aabb);

		update(target.parent);
	}

	void Tree::update(Body* body)
	{
		assert(body != nullptr);
		auto iter = m_leaves.find(body);
		if (iter == m_leaves.end())
			return;
		
		AABB thin = AABB::fromBody(body);
		thin.expand(0.2);
		if (!thin.isSubset(m_tree[iter->second].pair.aabb))
		{
			erase(body);
			insert(body);
		}
	}

	std::vector<Tree::Node> Tree::tree()
	{
		return m_tree;
	}

	void Tree::balance(int targetIndex)
	{
		
	}
	int Tree::level()
	{
		if (m_tree.empty())
			return 0;
		return static_cast<int>(std::floor(log2(m_tree.size()))) + 1;
	}
	int Tree::height(int index)
	{
		return index >= m_tree.size() || m_tree[index].isEmpty() ? 0 : Math::max(height(m_tree[index].left), height(m_tree[index].right) + 1);
	}
	void Tree::expand(int levels)
	{
		if(m_tree.empty())
		{
			m_tree.emplace_back(Node(0));
			return;
		}
		int targetLevel = level() + 1;
		int start = m_tree.size();
		for(int i = 0;i < pow(2, targetLevel - 1);i++)
			m_tree.emplace_back(Node(start + i));
	}

	int Tree::allocateNode()
	{
		if (!m_tree[m_tree.size() - 1].isEmpty())
			expand(1);

		for (auto& node : m_tree)
			if (node.isEmpty())
				return node.index;
	}

	bool Tree::isLeaf(int targetIndex)const
	{
		assert(targetIndex < m_tree.size());
		const Node& target = m_tree[targetIndex];
		if (target.left > m_tree.size() || target.right > m_tree.size())
			return true;
		return m_tree[target.left].isEmpty() && m_tree[target.right].isEmpty();
	}

	bool Tree::isBranch(int targetIndex)const
	{
		return !isLeaf(targetIndex);
	}

	bool Tree::isRoot(int targetIndex)const
	{
		assert(targetIndex < m_tree.size());
		const Node& target = m_tree[targetIndex];
		return m_tree[target.parent].isEmpty();
	}

	Tree::Node Tree::separate(int targetIndex, int sourceIndex)
	{
		assert(targetIndex < m_tree.size() && sourceIndex < m_tree.size());
		
		int child = -1;
		
		if (m_tree[targetIndex].left == sourceIndex)
			child = m_tree[targetIndex].right;
		else if (m_tree[targetIndex].right == sourceIndex)
			child = m_tree[targetIndex].left;
		else
			return Node();

		Node result = m_tree[sourceIndex];
		result.parent = -1;
		m_tree[sourceIndex].clear();

		Node& target = m_tree[targetIndex];
		target.pair = m_tree[child].pair;
		m_tree[child].clear();

		return result;
	}

	void Tree::merge(int targetIndex, const Pair& pair)
	{
		assert(targetIndex < m_tree.size());
		//make it parent node
		Node& targetNode = m_tree[targetIndex];
		Node& leftChild = m_tree[targetNode.left];
		Node& rightChild = m_tree[targetNode.right];

		leftChild.pair = targetNode.pair;
		rightChild.pair = pair;

		targetNode.pair.body = nullptr;
		targetNode.pair.aabb = AABB::unite(leftChild.pair.aabb, rightChild.pair.aabb);

		if(leftChild.pair.body != nullptr)
			m_leaves[leftChild.pair.body] = leftChild.index;

		if (rightChild.pair.body != nullptr)
			m_leaves[rightChild.pair.body] = rightChild.index;
		

		
	}
	real Tree::deltaCost(int nodeIndex, const AABB& aabb) const
	{
		assert(nodeIndex < m_tree.size());
		if (nodeIndex < 0)
			return 0;
		
		if (isLeaf(nodeIndex))
			return AABB::unite(m_tree[nodeIndex].pair.aabb, aabb).surfaceArea();

		return AABB::unite(m_tree[nodeIndex].pair.aabb, aabb).surfaceArea() - m_tree[nodeIndex].pair.aabb.surfaceArea();
	}
	real Tree::totalCost(int nodeIndex, const AABB& aabb) const
	{
		assert(nodeIndex < m_tree.size());
		real cost = 0;
		if (nodeIndex < 0)
			return cost;

		cost += deltaCost(nodeIndex, aabb);
		int parentIndex = m_tree[nodeIndex].parent;
		while(true)
		{
			if (parentIndex == -1)
				return cost;
			cost += deltaCost(parentIndex, aabb);
			parentIndex = m_tree[parentIndex].parent;
		}
	}
}
