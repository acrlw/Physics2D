#include "include/collision/broadphase/dbvt.h"
namespace Physics2D
{
	bool DBVT::Node::isLeaf() const
	{
		return parentIndex != -1 && leftIndex == -1 && rightIndex == -1;
	}

	bool DBVT::Node::isBranch() const
	{
		return parentIndex != -1 && leftIndex != -1 && rightIndex != -1;
	}

	bool DBVT::Node::isRoot() const
	{
		return parentIndex == -1 && leftIndex != -1 && rightIndex != -1;
	}

	bool DBVT::Node::isEmpty() const
	{
		return aabb.isEmpty();
	}

	void DBVT::Node::clear()
	{
		body = nullptr;
		aabb.clear();
		parentIndex = -1;
		leftIndex = -1;
		rightIndex = -1;
	}

	DBVT::DBVT()
	{
		m_tree.reserve(10);
	}

	std::vector<std::pair<Body*, Body*>> DBVT::generatePairs()
	{
		std::vector<std::pair<Body*, Body*>> result;
		return result;
	}

	void DBVT::insert(Body* body)
	{
		
	}
	void DBVT::remove(Body* body)
	{
		auto iter = m_bodyTable.find(body);
		if (iter == m_bodyTable.end())
			return;
		remove(iter->second);
		m_bodyTable.erase(iter);
	}

	std::vector<DBVT::Node>& DBVT::tree()
	{
		return m_tree;
	}

	int DBVT::rootIndex()const
	{
		return m_rootIndex;
	}

	void DBVT::extract(int targetIndex)
	{
		int parentIndex = m_tree[targetIndex].parentIndex;
		int anotherChildIndex = m_tree[parentIndex].leftIndex == targetIndex ? m_tree[parentIndex].rightIndex : m_tree[parentIndex].leftIndex;
		separate(targetIndex, parentIndex);
		elevate(anotherChildIndex);
		m_bodyTable[m_tree[targetIndex].body] = -1;
		remove(targetIndex);
	}

	int DBVT::merge(int sourceIndex, int targetIndex)
	{
		int parentIndex = allocateNode();
		m_tree[sourceIndex].parentIndex = parentIndex;
		m_tree[targetIndex].parentIndex = parentIndex;
		m_tree[parentIndex].leftIndex = sourceIndex;
		m_tree[parentIndex].rightIndex = targetIndex;
		m_tree[parentIndex].aabb = AABB::unite(m_tree[sourceIndex].aabb, m_tree[targetIndex].aabb);
		return parentIndex;

	}

	void DBVT::balance(int targetIndex)
	{

	}

	void DBVT::separate(int sourceIndex, int parentIndex)
	{
		if (m_tree[parentIndex].leftIndex == sourceIndex)
			m_tree[parentIndex].leftIndex = -1;
		else if (m_tree[parentIndex].rightIndex == sourceIndex)
			m_tree[parentIndex].rightIndex = -1;
		m_tree[sourceIndex].parentIndex = -1;

	}

	void DBVT::join(int nodeIndex, int boxIndex)
	{
		if (m_tree[boxIndex].leftIndex == -1)
			m_tree[boxIndex].leftIndex = nodeIndex;
		else if (m_tree[boxIndex].rightIndex == -1)
			m_tree[boxIndex].rightIndex = nodeIndex;
		m_tree[nodeIndex].parentIndex = boxIndex;
	}

	void DBVT::remove(int targetIndex)
	{
		m_tree[targetIndex].clear();
		m_emptyList.emplace_back(targetIndex);
	}

	void DBVT::elevate(int targetIndex)
	{
		int parentIndex = m_tree[targetIndex].parentIndex;
		int grandIndex = m_tree[parentIndex].parentIndex;
		separate(targetIndex, parentIndex);
		separate(parentIndex, grandIndex);
		join(targetIndex, grandIndex);
		remove(parentIndex);
	}

	void DBVT::updateNodeIndex(int newIndex)
	{
		
	}
	real DBVT::totalCost(int nodeIndex, int leafIndex)
	{
		real totalCost = AABB::unite(m_tree[nodeIndex].aabb, m_tree[leafIndex].aabb).surfaceArea();
		int currentIndex = m_tree[leafIndex].parentIndex;
		while(currentIndex != -1)
		{
			totalCost += deltaCost(nodeIndex, currentIndex);
			currentIndex = m_tree[currentIndex].parentIndex;
		}
		return totalCost;
	}
	real DBVT::deltaCost(int nodeIndex, int boxIndex)
	{
		return AABB::unite(m_tree[boxIndex].aabb, m_tree[nodeIndex].aabb).surfaceArea() - m_tree[nodeIndex].aabb.surfaceArea();
	}
	int DBVT::allocateNode()
	{
		if(!m_emptyList.empty())
		{
			int targetIndex = m_emptyList.back();
			m_emptyList.pop_back();
			return targetIndex;
		}
		m_tree.emplace_back(Node{});
		return m_tree.size() - 1;
	}

	int DBVT::height(int targetIndex)
	{
		return m_tree[targetIndex].isLeaf() ? 0 : std::max(height(m_tree[targetIndex].leftIndex), height(m_tree[targetIndex].rightIndex)) + 1;
	}
}
