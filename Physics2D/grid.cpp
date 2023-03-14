#include "grid.h"

namespace Physics2D
{
	UniformGrid::UniformGrid(const real& width, const real& height, const uint32_t rows, const uint32_t columns)
		:m_width(width), m_height(height), m_rows(rows), m_columns(columns)
	{
        updateGrid();
	}

    Container::Vector<std::pair<Body*, Body*>> UniformGrid::generate()
    {
        Container::Vector<std::pair<Body*, Body*>> result;
        Container::Map<Body::Relation::RelationID, std::pair<Body*, Body*>> map;
        for (auto&& cell : m_cellsToBodies)
        {
            if (cell.second.size() > 1)
            {
                for (auto iterOuter = cell.second.begin(); iterOuter != cell.second.end() - 1; ++iterOuter)
                {
                    for (auto iterInner = iterOuter + 1; iterInner != cell.second.end(); ++iterInner)
                    {
                        map[Body::Relation::generateRelationID(*iterInner, *iterOuter)] = std::make_pair(*iterInner, *iterOuter);
                    }
                }
            }
        }
        for (auto&& elem : map)
        {
            result.emplace_back(elem.second);
        }
        return result;
    }

    Container::Vector<Body*> UniformGrid::raycast(const Vector2& p, const Vector2& d)
    {
        Container::Vector<Body*> result;
        return result;
    }
    void UniformGrid::updateAll()
    {
        for (auto&& elem : m_bodiesToCells)
            update(elem.first);
        
    }
    void UniformGrid::update(Body* body)
    {
        //default option for update
        incrementalUpdate(body);
    }
    void UniformGrid::insert(Body* body)
    {
        assert(body != nullptr);
        auto iter = m_bodiesToCells.find(body);
        if (iter != m_bodiesToCells.end())
            return;
        auto cells = queryCells(body->aabb());
        m_bodiesToCells[body] = cells;
        for (auto&& elem : cells)
            m_cellsToBodies[elem].emplace_back(body);
        
    }

    void UniformGrid::remove(Body* body)
    {
        assert(body != nullptr);
        auto iter = m_bodiesToCells.find(body);
        if (iter == m_bodiesToCells.end())
            return;
    }

    int UniformGrid::rows() const
    {
        return m_rows;
    }

    void UniformGrid::setRows(const int& size)
    {
        m_rows = size;
        updateGrid();
    }

    int UniformGrid::columns() const
    {
        return m_columns;
    }

    void UniformGrid::setColumns(const int& size)
    {
        m_columns = size;
        updateGrid();
    }

    real UniformGrid::width() const
    {
        return m_width;
    }

    void UniformGrid::setWidth(const real& size)
    {
        m_width = size;
        updateGrid();
    }

    real UniformGrid::height() const
    {
        return m_height;
    }

    void UniformGrid::setHeight(const real& size)
    {
        m_height = size;
        updateGrid();
    }

    void UniformGrid::updateGrid()
    {
        changeGridSize();
        updateBodies();
    }

    void UniformGrid::changeGridSize()
    {
        assert(m_columns != 0 && m_rows != 0);
        m_cellWidth = m_width / real(m_columns);
        m_cellHeight = m_height / real(m_rows);
    }

    void UniformGrid::updateBodies()
    {

    }
    void UniformGrid::fullUpdate(Body* body)
    {
        assert(body != nullptr);
        auto iter = m_bodiesToCells.find(body);
        if (iter == m_bodiesToCells.end())
            return;
        auto cells = m_bodiesToCells[body];
        for (auto&& elem : cells) {
            auto& list = m_cellsToBodies[elem];
            auto bodyIter = std::find(list.begin(), list.end(), body);
            if (bodyIter != list.end())
                list.erase(bodyIter);
            if (list.empty())
                m_cellsToBodies.erase(elem);
        }
        m_bodiesToCells.erase(iter);
        insert(body);
    }
    void UniformGrid::incrementalUpdate(Body* body)
    {
        assert(body != nullptr);
        auto iter = m_bodiesToCells.find(body);
        if (iter == m_bodiesToCells.end())
            return;
        //Incremental update
        //cell list must be sorted array
        auto oldCellList = m_bodiesToCells[body];
        auto newCellList = queryCells(body->aabb());

        std::sort(oldCellList.begin(), oldCellList.end(), std::less<Position>());
        //New queryCellList has already been sorted.
        //std::sort(newCellList.begin(), newCellList.end(), std::less<Position>());
        auto changeList = compareCellList(oldCellList, newCellList);
        for (auto&& elem : changeList)
        {
            switch (elem.first)
            {
            case Operation::Add:
            {
                m_bodiesToCells[body].emplace_back(elem.second);
                m_cellsToBodies[elem.second].emplace_back(body);
                break;
            }
            case Operation::Delete:
            {
                auto& bodyList = m_cellsToBodies[elem.second];
                bodyList.erase(
                    std::remove_if(bodyList.begin(), bodyList.end(),
                        [&body](Body* target) { return body == target; }), bodyList.end());

                if (bodyList.empty())
                    m_cellsToBodies.erase(elem.second);

                auto& positionList = m_bodiesToCells[body];
                positionList.erase(
                    std::remove_if(positionList.begin(), positionList.end(),
                        [&elem](const Position& other) { return elem.second == other; }), positionList.end());

                break;
            }
            }
        }
    }
    Container::Vector<std::pair<UniformGrid::Operation, UniformGrid::Position>> UniformGrid::compareCellList(const Container::Vector<Position>& oldCellList, const Container::Vector<Position>& newCellList)
    {
        Container::Vector<std::pair<UniformGrid::Operation, UniformGrid::Position>> result;
        size_t indexOld = 0, indexNew = 0;
        size_t endIndexOld = oldCellList.size();
        size_t endIndexNew = newCellList.size();
        while (true)
        {
            if (indexOld == endIndexOld && indexNew == endIndexNew)
                break;
            if (indexNew < endIndexNew && indexOld == endIndexOld)
            {
                result.emplace_back(std::make_pair(Operation::Add, newCellList[indexNew]));
                ++indexNew;
                continue;
            }
            if (indexOld < endIndexOld && indexNew == endIndexNew)
            {
                result.emplace_back(std::make_pair(Operation::Delete, oldCellList[indexOld]));
                ++indexOld;
                continue;
            }
            if (indexOld < endIndexOld && indexNew < endIndexNew)
            {
                if (oldCellList[indexOld] == newCellList[indexNew])
                {
                    ++indexOld;
                    ++indexNew;
                    continue;
                }
                if (oldCellList[indexOld] > newCellList[indexNew])
                {
                    result.emplace_back(std::make_pair(Operation::Add, newCellList[indexNew]));
                    ++indexNew;
                    continue;
                }
                if (oldCellList[indexOld] < newCellList[indexNew])
                {
                    result.emplace_back(std::make_pair(Operation::Delete, oldCellList[indexOld]));
                    ++indexOld;
                    continue;
                }
            }
        }
        return result;
    }
    Container::Vector<UniformGrid::Position> UniformGrid::queryCells(const AABB& aabb)
    {
        Container::Vector<UniformGrid::Position> cells;
        //locate x axis
        real halfWidth = m_width * 0.5f;
        real halfHeight = m_height * 0.5f;
        real xRealMin = aabb.minimumX();
        real xRealMax = aabb.maximumX();
        real xMin = Math::clamp(xRealMin, -halfWidth, halfWidth - m_cellWidth);
        real xMax = Math::clamp(xRealMax, -halfWidth, halfWidth - m_cellWidth);
        real lowerXIndex = std::floor((xMin + halfWidth) / m_cellWidth);
        real upperXIndex = std::floor((xMax + halfWidth) / m_cellWidth);
        //locate y axis

        real yRealMin = aabb.minimumY();
        real yRealMax = aabb.maximumY();
        real yMin = Math::clamp(yRealMin, -halfHeight + m_cellHeight, halfHeight);
        real yMax = Math::clamp(yRealMax, -halfHeight + m_cellHeight, halfHeight);
        real lowerYIndex = std::ceil((yMin + halfHeight) / m_cellHeight);
        real upperYIndex = std::ceil((yMax + halfHeight) / m_cellHeight);
        if (lowerXIndex == upperXIndex && xRealMax < -halfWidth || xRealMin > halfWidth)
            return cells;
        if (lowerYIndex == upperYIndex && yRealMax < -halfHeight || yRealMin > halfHeight)
            return cells;

        for(real i = lowerXIndex; i <= upperXIndex; i += 1.0f)
        {
            for (real j = lowerYIndex; j <= upperYIndex; j += 1.0f)
            {
                cells.emplace_back(Position{ static_cast<uint32_t>(i), static_cast<uint32_t>(j) });
            }
        }

        return cells;

    }

    real UniformGrid::cellHeight() const
    {
        return m_cellHeight;
    }

    real UniformGrid::cellWidth() const
    {
        return m_cellWidth;
    }
}
