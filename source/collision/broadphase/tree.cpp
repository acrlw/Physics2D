#include "include/collision/broadphase/tree.h"
namespace Physics2D
{
	Tree::Node::Node(Body* b, int i):body(b), index(i)
	{
		left = 2 * i + 1;
		right = 2 * i + 2;
		parent = (i - 1) / 2;
	}
	bool Tree::Node::isEmpty() const
	{
		return aabb.isEmpty();
	}
	void Tree::Node::clear()
	{
		parent = -1;
		body = nullptr;
		aabb.clear();
	}
	void Tree::insert(Body* body)
	{
		assert(body != nullptr);
		
		if(std::find_if(m_tree.begin(), m_tree.end(), [body](const Node& node)
			{
				return node.body == body;
			}) != m_tree.end())
			return;

		auto assignNode = [&](int index, Body* body)
		{
			assert(index < m_tree.size());
			m_tree[index].body = body;
			AABB aabb = AABB::fromBody(body);
			aabb.expand(m_leafFactor);
			m_tree[index].aabb = aabb;
		};
		auto getCost = [&](int index, const AABB& aabb)
		{
			if(isLeaf(index))
				return std::make_tuple(index, AABB::unite(m_tree[index].aabb, aabb).surfaceArea());

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
			return std::make_tuple(targetIndex, minimumCost);
		};

		
		if(m_leaves.empty())
		{
			expand(1);
			assignNode(0, body);
			m_leaves[body] = 0;
			return;
		}
		

		
	}

	void Tree::erase(Body* body)
	{
		
	}

	void Tree::extract(Body* body)
	{
		
	}

	void Tree::update(Body* body)
	{
		
	}

	void Tree::balance(int targetIndex)
	{
	}

	void Tree::update(int targetIndex)
	{
	}

	void Tree::expand(int levels)
	{
		if(m_tree.empty())
		{
			
		}
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
		target.aabb = m_tree[child].aabb;
		target.body = m_tree[child].body;
		m_tree[child].clear();

		return result;
	}

	void Tree::merge(int targetIndex, const Node& node)
	{
		assert(targetIndex < m_tree.size());
		//make it parent node
		Node& targetNode = m_tree[targetIndex];
		Node& leftChild = m_tree[targetNode.left];
		Node& rightChild = m_tree[targetNode.right];
		leftChild.body = targetNode.body;
		leftChild.aabb = targetNode.aabb;
		rightChild.body = node.body;
		rightChild.aabb = node.aabb;
		
		targetNode.body = nullptr;
		targetNode.aabb = AABB::unite(leftChild.aabb, rightChild.aabb);
	}
	real Tree::deltaCost(int nodeIndex, const AABB& aabb) const
	{
		assert(nodeIndex < m_tree.size());
		if (nodeIndex < 0)
			return 0;
		
		if (isLeaf(nodeIndex))
			return AABB::unite(m_tree[nodeIndex].aabb, aabb).surfaceArea();

		return AABB::unite(m_tree[nodeIndex].aabb, aabb).surfaceArea() - m_tree[nodeIndex].aabb.surfaceArea();
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
