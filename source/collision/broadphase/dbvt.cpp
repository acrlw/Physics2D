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
	}

	void DBVT::merge(int targetIndex)
	{
	}

	void DBVT::balance(int targetIndex)
	{
	}
	void DBVT::updateNodeIndex(int targetIndex)
	{
		
	}
	int DBVT::allocateNode()
	{
		if(!m_emptyList.empty())
		{
			int targetIndex = m_emptyList[0];
			m_emptyList.erase(m_emptyList.begin());
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
