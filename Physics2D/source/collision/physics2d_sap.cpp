#include "physics2d_sap.h"

#include "physics2d_body.h"

namespace Physics2D
{
	Container::Vector<std::pair<Body*, Body*>> SweepAndPrune::generate(const Container::Vector<Body*>& bodyList)
	{
		Container::Vector<std::pair<Body*, Body*>> result;
		//sort by x axis
		Container::Vector<std::pair<Body*, AABB>> bodyBoxPairList;
		bodyBoxPairList.reserve(bodyList.size());

		for(auto&& elem: bodyList)
			bodyBoxPairList.emplace_back(std::make_pair(elem, elem->aabb()));
		
		Container::Vector<std::pair<Body*, AABB>> sortXAxis = bodyBoxPairList;
		Container::Vector<std::pair<Body*, AABB>> sortYAxis = bodyBoxPairList;
		std::sort(sortXAxis.begin(), sortXAxis.end(), [](const std::pair<Body*, AABB>& left, const std::pair<Body*, AABB>& right)
			{
				return left.second.minimumX() < right.second.minimumX();
			});
		std::sort(sortYAxis.begin(), sortYAxis.end(), [](const std::pair<Body*, AABB>& left, const std::pair<Body*, AABB>& right)
			{
				return left.second.minimumY() < right.second.minimumY();
			});

		
		Container::Vector<Body::BodyPair> xPairs;
		Container::Vector<Body::BodyPair> yPairs;

		for (auto before = sortXAxis.begin(); before != sortXAxis.end(); ++before)
		{
			for (auto next = std::next(before); next != sortXAxis.end(); ++next)
			{
				const real minBefore = before->second.minimumX();
				const real maxBefore = before->second.maximumX();
				const real minNext = next->second.minimumX();
				const real maxNext = next->second.maximumX();

				if (!(maxBefore < minNext || maxNext < minBefore))
					xPairs.emplace_back(Body::BodyPair::generateBodyPair(before->first, next->first));
				else
					break;
			}
		}

		for (auto before = sortYAxis.begin(); before != sortYAxis.end(); ++before)
		{
			for (auto next = std::next(before); next != sortYAxis.end(); ++next)
			{
				const real minBefore = before->second.minimumY();
				const real maxBefore = before->second.maximumY();
				const real minNext = next->second.minimumY();
				const real maxNext = next->second.maximumY();

				if (!(maxBefore < minNext || maxNext < minBefore))
					yPairs.emplace_back(Body::BodyPair::generateBodyPair(before->first, next->first));
				else
					break;
			}
		}

		std::sort(xPairs.begin(), xPairs.end(), [](const Body::BodyPair& left, const Body::BodyPair& right)
			{
				return left.pairID < right.pairID;
			});
		std::sort(yPairs.begin(), yPairs.end(), [](const Body::BodyPair& left, const Body::BodyPair& right)
			{
				return left.pairID < right.pairID;
			});

		//double pointer check
		auto xPair = xPairs.begin();
		auto yPair = yPairs.begin();
		while (xPair != xPairs.end() && yPair != yPairs.end())
		{
			if (xPair->pairID == yPair->pairID && (xPair->bodyA->bitmask() & xPair->bodyB->bitmask()))
			{
				result.emplace_back(std::make_pair(xPair->bodyA, xPair->bodyB));
				xPair = std::next(xPair);
				yPair = std::next(yPair);
			}
			else if(xPair->pairID > yPair->pairID)
				yPair = std::next(yPair);
			else
				xPair = std::next(xPair);
		}


		return result;
	}

	Container::Vector<Body*> SweepAndPrune::query(const Container::Vector<Body*>& bodyList, const AABB& region)
	{
		Container::Vector<Body*> result;

		Container::Vector<std::pair<Body*, AABB>> bodyBoxPairList;
		bodyBoxPairList.reserve(bodyList.size());

		for (auto&& elem : bodyList)
			bodyBoxPairList.emplace_back(std::make_pair(elem, elem->aabb()));

		//brute search
		for (auto&& elem : bodyBoxPairList)
			if (region.collide(elem.second))
				result.emplace_back(elem.first);



		return result;
	}
}
