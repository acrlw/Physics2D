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
    		switch (body->shape()->type())
	        {
            case Shape::Type::Polygon:
	            {
				RendererQtImpl::renderPolygon(painter, world, primitive, pen);
				break;
	            }
            case Shape::Type::Ellipse:
	            {
				RendererQtImpl::renderEllipse(painter, world, primitive, pen);
				break;
	            }
            case Shape::Type::Circle:
	            {
				RendererQtImpl::renderCircle(painter, world, primitive, pen);
				break;
	            }
            case Shape::Type::Curve:
	            {
				RendererQtImpl::renderCurve(painter, world, primitive, pen);
				break;
	            }
            case Shape::Type::Edge:
	            {
				RendererQtImpl::renderEdge(painter, world, primitive, pen);
				break;
	            }
            default: break;
	        }
    	}
    }
}
