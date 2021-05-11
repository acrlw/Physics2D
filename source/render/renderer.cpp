#include "include/render/renderer.h"
#include "include/render/impl/renderer_qt.h"
namespace Physics2D
{
    void Renderer::render(QPainter* painter, World* world, const QPen& pen)
    {
    	for(Body * body: world->bodyList())
			render(painter, world, body, pen);
    	
		for (Joint* joint : world->jointList())
			RendererQtImpl::renderJoint(painter, world, joint, pen);
    }
	void Renderer::render(QPainter* painter, World* world, Body* body, const QPen& pen)
	{
		ShapePrimitive primitive;
		primitive.shape = body->shape();
		primitive.rotation = body->angle();
		primitive.transform = body->position();
		RendererQtImpl::renderShape(painter, world, primitive, pen);
	}
}
