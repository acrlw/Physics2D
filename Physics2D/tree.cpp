#include "tree.h"

#include "body.h"

namespace Physics2D
{
	bool Tree::Node::isLeaf() const
	{
		return leftIndex == -1 && rightIndex == -1;
	}

	bool Tree::Node::isBranch() const
	{
		return parentIndex != -1 && leftIndex != -1 && rightIndex != -1;
	}

	bool Tree::Node::isRoot() const
	{
		return parentIndex == -1 && leftIndex != -1 && rightIndex != -1;
	}

	bool Tree::Node::isEmpty() const
	{
		return aabb.isEmpty();
	}

	void Tree::Node::clear()
	{
		body = nullptr;
		aabb.clear();
		parentIndex = -1;
		leftIndex = -1;
		rightIndex = -1;
	}

	Tree::Tree()
	{
		m_tree.reserve(10);
	}

	Container::Vector<Body*> Tree::query(Body* body)
	{
		Container::Vector<Body*> result;
		queryNodes(m_rootIndex, AABB::fromBody(body), result);
		return result;
	}

	Container::Vector<Body*> Tree::query(const AABB& aabb)
	{
		Container::Vector<Body*> result;
		queryNodes(m_rootIndex, aabb, result);
		return result;
	}

	Container::Vector<Body*> Tree::raycast(const Vector2& point, const Vector2& direction)
	{
		Container::Vector<Body*> result;
		raycast(result, m_rootIndex, point, direction);
		return result;
	}

	Container::Vector<std::pair<Body*, Body*>> Tree::generate()
	{
		Container::Vector<std::pair<Body*, Body*>> pairs;
		generate(m_rootIndex, pairs);
		return pairs;
	}
	

	void Tree::insert(Body* body)
	{
		int newNodeIndex = allocateNode();
		m_tree[newNodeIndex].body = body;
		m_tree[newNodeIndex].aabb = AABB::fromBody(body);
		m_tree[newNodeIndex].aabb.expand(m_fatExpansionFactor);
		m_bodyTable[body] = newNodeIndex;
		if(m_rootIndex == -1)
		{
			m_rootIndex = newNodeIndex;
			return;
		}
		if(m_tree[m_rootIndex].isLeaf())
		{
			m_rootIndex = merge(newNodeIndex, m_rootIndex);
			return;
		}

		//calc cost
		int targetIndex = calculateLowestCostNode(newNodeIndex);

		if(targetIndex == m_rootIndex)
		{
			m_rootIndex = merge(newNodeIndex, targetIndex);
			balance(m_rootIndex);
			return;
		}

		int targetParentIndex = m_tree[targetIndex].parentIndex;
		separate(targetIndex, targetParentIndex);
		int boxIndex = merge(newNodeIndex, targetIndex);
		join(boxIndex, targetParentIndex);
		upgrade(boxIndex);
		balance(m_rootIndex);

	}
	void Tree::remove(Body* body)
	{
		auto iter = m_bodyTable.find(body);
		if (iter == m_bodyTable.end())
			return;
		int parentIndex = m_tree[iter->second].parentIndex;

		if (parentIndex == -1 && m_tree[iter->second].isLeaf())
		{
			m_rootIndex = -1;
			remove(iter->second);
			m_bodyTable.erase(iter);
			return;
		}

		int anotherChild = m_tree[parentIndex].leftIndex == iter->second ? m_tree[parentIndex].rightIndex : m_tree[parentIndex].leftIndex;
		remove(iter->second);
		elevate(anotherChild);
		upgrade(anotherChild);
		m_bodyTable.erase(iter);
	}

	void Tree::clearAll()
	{
		m_tree.clear();
		m_emptyList.clear();
		m_bodyTable.clear();
		m_rootIndex = -1;
	}

	void Tree::update(Body* body)
	{
		auto iter = m_bodyTable.find(body);
		if (iter == m_bodyTable.end())
			return;

		AABB thin = AABB::fromBody(body);
		thin.expand(0.1f);
		if (!thin.isSubset(m_tree[iter->second].aabb))
		{
			extract(iter->second);
			insert(body);
		}
	}

	int Tree::rootIndex() const
	{
		return m_rootIndex;
	}

	const Container::Vector<Tree::Node>& Tree::tree()
	{
		return m_tree;
	}

	void Tree::queryNodes(int nodeIndex, const AABB& aabb, Container::Vector<Body*>& result)
	{
		if (nodeIndex == -1)
			return;
		const bool overlap = m_tree[nodeIndex].aabb.collide(aabb) || m_tree[nodeIndex].aabb.isSubset(aabb)
			|| aabb.isSubset(m_tree[nodeIndex].aabb);

		if (!overlap)
			return;

		if (m_tree[nodeIndex].isLeaf())
		{
			result.emplace_back(m_tree[nodeIndex].body);
			return;
		}
		if (m_tree[nodeIndex].isBranch() || m_tree[nodeIndex].isRoot())
		{
			queryNodes(m_tree[nodeIndex].leftIndex, aabb, result);
			queryNodes(m_tree[nodeIndex].rightIndex, aabb, result);
		}

	}

	void Tree::traverseLowestCost(int nodeIndex, int boxIndex, real& cost, int& finalIndex)
	{
		//Search for best leaf node
		
		//if (m_tree[boxIndex].isLeaf())
		//{
		//	finalIndex = boxIndex;
		//	return;
		//}

		//if (boxIndex == m_rootIndex)
		//{
		//	finalIndex = -1;
		//	cost = deltaCost(nodeIndex, boxIndex);
		//}

		//auto accumulateCost = [&](int nodeIndex, int boxIndex)
		//{
		//	if (m_tree[nodeIndex].isLeaf())
		//		return cost + AABB::unite(m_tree[nodeIndex].aabb, m_tree[boxIndex].aabb).surfaceArea();
		//	return deltaCost(nodeIndex, boxIndex);
		//};

		//int leftIndex = m_tree[boxIndex].leftIndex;
		//int rightIndex = m_tree[boxIndex].rightIndex;
		//real leftCost = accumulateCost(nodeIndex, leftIndex);
		//real rightCost = accumulateCost(nodeIndex, rightIndex);
		//real lowestCost;
		//int lowestCostIndex;
		//if(leftCost > rightCost)
		//{
		//	lowestCost = rightCost;
		//	lowestCostIndex = rightIndex;
		//}
		//else
		//{
		//	lowestCost = leftCost;
		//	lowestCostIndex = leftIndex;
		//}
		//traverseLowestCost(nodeIndex, lowestCostIndex, lowestCost, finalIndex);

		//Branch And Bound from box2d
		//A little bit faster because it doesn't always reach leaf node when traversing.
		//The final bvh result is worse than brute search, but collision pair generation is almost the same, only 100 less than brute search

		if (m_tree[boxIndex].isLeaf() && !m_tree[boxIndex].isRoot())
		{
			finalIndex = boxIndex;
			return;
		}

		real area = m_tree[boxIndex].aabb.surfaceArea();
		real unionArea = AABB::unite(m_tree[nodeIndex].aabb, m_tree[boxIndex].aabb).surfaceArea();

		cost = 2.0f * area;
		real inheritanceCost = 2.0f * (unionArea - area);

		auto accumulateCost = [&](int nodeIndex, int boxIndex)
		{
			if (m_tree[nodeIndex].isLeaf())
				return inheritanceCost + AABB::unite(m_tree[nodeIndex].aabb, m_tree[boxIndex].aabb).surfaceArea();
			return deltaCost(nodeIndex, boxIndex) + inheritanceCost;
		};

		int leftIndex = m_tree[boxIndex].leftIndex;
		int rightIndex = m_tree[boxIndex].rightIndex;
		real leftCost = accumulateCost(nodeIndex, leftIndex);
		real rightCost = accumulateCost(nodeIndex, rightIndex);
		real lowestCost;
		int lowestCostIndex;

		if(cost < leftCost && cost < rightCost)
		{
			finalIndex = boxIndex;
			return;
		}

		if(leftCost > rightCost)
		{
			lowestCost = rightCost;
			lowestCostIndex = rightIndex;
		}
		else
		{
			lowestCost = leftCost;
			lowestCostIndex = leftIndex;
		}
		finalIndex = lowestCostIndex;
		traverseLowestCost(nodeIndex, lowestCostIndex, lowestCost, finalIndex);
	}

	void Tree::raycast(Container::Vector<Body*>& result, int nodeIndex, const Vector2& p, const Vector2& d)
	{
		if (nodeIndex < 0)
			return;

		if(m_tree[nodeIndex].aabb.raycast(p, d))
		{
			if (m_tree[nodeIndex].isLeaf())
				result.emplace_back(m_tree[nodeIndex].body);
			else
			{
				raycast(result, m_tree[nodeIndex].leftIndex, p, d);
				raycast(result, m_tree[nodeIndex].rightIndex, p, d);
			}
		}
	}

	void Tree::generate(int nodeIndex, Container::Vector<std::pair<Body*, Body*>>& pairs)
	{
		if (nodeIndex < 0 || m_tree[nodeIndex].isLeaf())
			return;

		int leftIndex = m_tree[nodeIndex].leftIndex;
		int rightIndex = m_tree[nodeIndex].rightIndex;
		bool result = m_tree[leftIndex].aabb.collide(m_tree[rightIndex].aabb);
		
		if (result)
			generate(leftIndex, rightIndex, pairs);

		generate(leftIndex, pairs);
		generate(rightIndex, pairs);
	}

	void Tree::generate(int leftIndex, int rightIndex, Container::Vector<std::pair<Body*, Body*>>& pairs)
	{
		if (leftIndex < 0 || rightIndex < 0)
			return;

		bool result = m_tree[leftIndex].aabb.collide(m_tree[rightIndex].aabb) || m_tree[leftIndex].aabb.isSubset(m_tree[rightIndex].aabb)
		|| m_tree[rightIndex].aabb.isSubset(m_tree[leftIndex].aabb);

		if (!result)
			return;

		if (m_tree[leftIndex].isLeaf() && m_tree[rightIndex].isLeaf())
		{
			if(m_tree[leftIndex].body->bitmask() & m_tree[rightIndex].body->bitmask())
			{
				//if AABB of A & B overlap
				if (AABB::fromBody(m_tree[leftIndex].body).collide(AABB::fromBody(m_tree[rightIndex].body)))
				{
					std::pair pair = { m_tree[leftIndex].body, m_tree[rightIndex].body };
					pairs.emplace_back(pair);
				}
			}
		}
		if (m_tree[leftIndex].isLeaf() && m_tree[rightIndex].isBranch())
		{
			generate(leftIndex, m_tree[rightIndex].leftIndex, pairs);
			generate(leftIndex, m_tree[rightIndex].rightIndex, pairs);
		}
		if (m_tree[rightIndex].isLeaf() && m_tree[leftIndex].isBranch())
		{
			generate(rightIndex, m_tree[leftIndex].leftIndex, pairs);
			generate(rightIndex, m_tree[leftIndex].rightIndex, pairs);
		}
		if (m_tree[leftIndex].isBranch() && m_tree[rightIndex].isBranch())
		{
			generate(m_tree[leftIndex].leftIndex, rightIndex, pairs);
			generate(m_tree[leftIndex].rightIndex, rightIndex, pairs);
		}
	}

	void Tree::extract(int targetIndex)
	{
		if(targetIndex == m_rootIndex)
		{
			m_rootIndex = -1;
			m_bodyTable[m_tree[targetIndex].body] = -1;
			remove(targetIndex);
			return;
		}
		if(m_tree[targetIndex].parentIndex == m_rootIndex)
		{
			int anotherChildIndex = m_tree[m_rootIndex].leftIndex == targetIndex ? m_tree[m_rootIndex].rightIndex : m_tree[m_rootIndex].leftIndex;
			separate(targetIndex, m_rootIndex);
			elevate(anotherChildIndex);
			m_bodyTable[m_tree[targetIndex].body] = -1;
			remove(targetIndex);
			return;
		}
		int parentIndex = m_tree[targetIndex].parentIndex;
		int anotherChildIndex = m_tree[parentIndex].leftIndex == targetIndex ? m_tree[parentIndex].rightIndex : m_tree[parentIndex].leftIndex;
		separate(targetIndex, parentIndex);
		elevate(anotherChildIndex);
		m_bodyTable[m_tree[targetIndex].body] = -1;
		remove(targetIndex);
	}

	int Tree::merge(int nodeIndex, int leafIndex)
	{
		int parentIndex = allocateNode();
		m_tree[leafIndex].parentIndex = parentIndex;
		m_tree[nodeIndex].parentIndex = parentIndex;
		m_tree[parentIndex].leftIndex = leafIndex;
		m_tree[parentIndex].rightIndex = nodeIndex;
		m_tree[parentIndex].aabb = AABB::unite(m_tree[nodeIndex].aabb, m_tree[leafIndex].aabb);
		return parentIndex;

	}

	void Tree::rr(int nodeIndex)
	{
		if (nodeIndex == -1 || m_tree[nodeIndex].isRoot())
			return;

		if (m_tree[nodeIndex].parentIndex == m_rootIndex)
		{
			int parentIndex = m_tree[nodeIndex].parentIndex;
			int leftIndex = m_tree[nodeIndex].leftIndex;
			separate(nodeIndex, parentIndex);
			separate(leftIndex, nodeIndex);
			join(leftIndex, parentIndex);
			join(parentIndex, nodeIndex);
			m_rootIndex = nodeIndex;
			upgrade(parentIndex);
			return;
		}
		int parentIndex = m_tree[nodeIndex].parentIndex;
		int grandIndex = m_tree[parentIndex].parentIndex;
		int leftIndex = m_tree[nodeIndex].leftIndex;
		separate(parentIndex, grandIndex);
		separate(nodeIndex, parentIndex);
		separate(leftIndex, nodeIndex);
		join(leftIndex, parentIndex);
		join(parentIndex, nodeIndex);
		join(nodeIndex, grandIndex);
		upgrade(parentIndex);
	}

	void Tree::ll(int nodeIndex)
	{
		if (nodeIndex == -1 || m_tree[nodeIndex].isRoot())
			return;

		if (m_tree[nodeIndex].parentIndex == m_rootIndex)
		{
			int parentIndex = m_tree[nodeIndex].parentIndex;
			int rightIndex = m_tree[nodeIndex].rightIndex;
			separate(nodeIndex, parentIndex);
			separate(rightIndex, nodeIndex);
			join(rightIndex, parentIndex);
			join(parentIndex, nodeIndex);
			m_rootIndex = nodeIndex;
			upgrade(parentIndex);
			return;
		}
		int parentIndex = m_tree[nodeIndex].parentIndex;
		int grandIndex = m_tree[parentIndex].parentIndex;
		int rightIndex = m_tree[nodeIndex].rightIndex;
		separate(parentIndex, grandIndex);
		separate(nodeIndex, parentIndex);
		separate(rightIndex, nodeIndex);
		join(rightIndex, parentIndex);
		join(parentIndex, nodeIndex);
		join(nodeIndex, grandIndex);
		upgrade(parentIndex);
	}

	void Tree::balance(int targetIndex)
	{
		if (targetIndex == -1 || m_tree[targetIndex].isLeaf())
			return;

		const int leftHeight = height(m_tree[targetIndex].leftIndex);
		const int rightHeight = height(m_tree[targetIndex].rightIndex);
		if (std::fabs(leftHeight - rightHeight) <= 1)
			return;

		if (leftHeight > rightHeight) //left unbalance
		{
			const int leftLeftHeight = height(m_tree[m_tree[targetIndex].leftIndex].leftIndex);
			const int leftRightHeight = height(m_tree[m_tree[targetIndex].leftIndex].rightIndex);
			if (leftLeftHeight < leftRightHeight) //LR case
				rr(m_tree[m_tree[targetIndex].leftIndex].rightIndex);
			else
				ll(m_tree[m_tree[targetIndex].leftIndex].leftIndex);
			ll(m_tree[targetIndex].leftIndex);
		}
		else //right unbalance
		{
			const int rightRightHeight = height(m_tree[m_tree[targetIndex].rightIndex].rightIndex);
			const int rightLeftHeight = height(m_tree[m_tree[targetIndex].rightIndex].leftIndex);
			if (rightRightHeight < rightLeftHeight) //RL case
				ll(m_tree[m_tree[targetIndex].rightIndex].leftIndex);
			else
				rr(m_tree[m_tree[targetIndex].rightIndex].rightIndex);
			rr(m_tree[targetIndex].rightIndex);
		}
		balance(m_tree[targetIndex].leftIndex);
		balance(m_tree[targetIndex].rightIndex);
		balance(m_tree[targetIndex].parentIndex);
	}

	void Tree::separate(int sourceIndex, int parentIndex)
	{
		if (sourceIndex < 0 || parentIndex < 0)
			return;

		if (m_tree[parentIndex].leftIndex == sourceIndex)
			m_tree[parentIndex].leftIndex = -1;
		else if (m_tree[parentIndex].rightIndex == sourceIndex)
			m_tree[parentIndex].rightIndex = -1;
		m_tree[sourceIndex].parentIndex = -1;
	}

	void Tree::join(int nodeIndex, int boxIndex)
	{
		if (nodeIndex < 0 || boxIndex < 0)
			return;

		if (m_tree[boxIndex].leftIndex == -1)
			m_tree[boxIndex].leftIndex = nodeIndex;
		else if (m_tree[boxIndex].rightIndex == -1)
			m_tree[boxIndex].rightIndex = nodeIndex;
		m_tree[nodeIndex].parentIndex = boxIndex;
	}

	void Tree::remove(int targetIndex)
	{
		m_tree[targetIndex].clear();
		m_emptyList.emplace_back(targetIndex);
	}

	void Tree::elevate(int targetIndex)
	{
		if(m_tree[targetIndex].parentIndex == m_rootIndex)
		{
			remove(m_rootIndex);
			m_rootIndex = targetIndex;
			m_tree[targetIndex].parentIndex = -1;
			return;
		}
		int parentIndex = m_tree[targetIndex].parentIndex;
		int grandIndex = m_tree[parentIndex].parentIndex;
		separate(targetIndex, parentIndex);
		separate(parentIndex, grandIndex);
		join(targetIndex, grandIndex);
		remove(parentIndex);
	}
	int Tree::calculateLowestCostNode(int nodeIndex)
	{
		real lowestCost = Constant::Max;
		int targetIndex = -1;
		//start traverse lowest cost node
		traverseLowestCost(nodeIndex, m_rootIndex, lowestCost, targetIndex);

		//brute search every leaf to find the lowest cost
		
		//for (auto& [body, leafIndex] : m_bodyTable)
		//{
		//	if (leafIndex == nodeIndex)
		//		continue;
		//	real cost = totalCost(nodeIndex, leafIndex);
		//	if (lowestCost > cost)
		//	{
		//		lowestCost = cost;
		//		targetIndex = leafIndex;
		//	}
		//}

		return targetIndex;
	}
	real Tree::totalCost(int nodeIndex, int leafIndex)
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
	void Tree::upgrade(int nodeIndex)
	{
		if (nodeIndex < 0 || m_tree[nodeIndex].isLeaf())
			return;
		
		m_tree[nodeIndex].aabb = AABB::unite(m_tree[m_tree[nodeIndex].leftIndex].aabb, m_tree[m_tree[nodeIndex].rightIndex].aabb);

		upgrade(m_tree[nodeIndex].parentIndex);
	}
	real Tree::deltaCost(int nodeIndex, int boxIndex)
	{
		return AABB::unite(m_tree[boxIndex].aabb, m_tree[nodeIndex].aabb).surfaceArea() - m_tree[boxIndex].aabb.surfaceArea();
	}
	size_t Tree::allocateNode()
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

	int Tree::height(int targetIndex)
	{
		return targetIndex < 0 ? 0 : std::max(height(m_tree[targetIndex].leftIndex), height(m_tree[targetIndex].rightIndex)) + 1;
	}
}
