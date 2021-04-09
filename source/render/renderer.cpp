#include "include/render/renderer.h"
#include "include/render/impl/renderer_qt.h"
namespace Physics2D
{
    void Renderer::render(QPainter* painter, World* world, const QPen& pen)
    {
    	for(const Body* body: world->bodyList())
    	{
			ShapePrimitive primitive;
			primitive.shape = body->shape();
			primitive.rotation = body->angle();
			primitive.transform = body->position();
            RendererQtImpl::renderShape(painter, world, primitive, pen);
    	}
    }
}
