#include "../../../include/collision/broadphase/grid.h"

namespace Physics2D
{
    std::vector<std::pair<Body*, Body*>> UniformGrid::generate()
    {
        std::vector<std::pair<Body*, Body*>> result;
        return result;
    }

    std::vector<Body*> UniformGrid::raycast(const Vector2& p, const Vector2& d)
    {
        std::vector<Body*> result;
        return result;
    }

    void UniformGrid::update(Body* body)
    {
    }
    void UniformGrid::insert(Body* body)
    {
    }

    void UniformGrid::remove(Body* body)
    {
    }
}
