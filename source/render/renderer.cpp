#include "include/render/renderer.h"
#include "include/render/impl/renderer_qt.h"
namespace Physics2D
{
    void Renderer::render(QPainter* painter, World* world, const QPen& pen)
    {
		if (painter == nullptr || world == nullptr)
			return;
    	
		for(auto& body: world->bodyList())
			render(painter, world, body.get(), pen);

		for (auto& joint : world->jointList())
			RendererQtImpl::renderJoint(painter, world, joint.get(), pen);
    }
	void Renderer::render(QPainter* painter, World* world, Body* body, const QPen& pen)
	{
		if (painter == nullptr || world == nullptr || body == nullptr)
			return;
		ShapePrimitive primitive;
		primitive.shape = body->shape();
		primitive.rotation = body->angle();
		primitive.transform = body->position();
		RendererQtImpl::renderShape(painter, world, primitive, pen);
	}
}
