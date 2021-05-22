#include "include/utils/camera.h"

#include "include/render/impl/renderer_qt.h"

namespace Physics2D::Utils
{
	void Camera::render(QPainter* painter)
	{
		if(m_visible)
		{
			assert(painter != nullptr && m_world != nullptr);


			if (m_targetBody != nullptr)
			{
				real dt = 1.0 / 30.0;
				Vector2 real_origin(m_origin.x + m_transform.x, m_origin.y - m_transform.y);
				Vector2 target(-m_targetBody->position().x, m_targetBody->position().y);
				target = worldToScreen(target) - real_origin;
				
				//lerp
				//if (c.lengthSquare() < 0.1)
				//	m_transform = target;
				//else
				//	m_transform += c * 0.02;

				//exp
				Vector2 c = target - m_transform;
				if (c.lengthSquare() < 0.1)
					m_transform = target;
				else
					m_transform -= (1.0 - std::exp(dt)) * c;
				
				//real frames = m_easingTime * 60;
				
			}

			
			painter->setRenderHint(QPainter::Antialiasing);
			painter->setClipRect(m_viewport.topLeft.x, m_viewport.topLeft.y, m_viewport.width(), m_viewport.height());
			painter->setBackground(QBrush(QColor(50, 50, 50)));
			painter->fillRect(QRectF(m_viewport.topLeft.x, m_viewport.topLeft.y, m_viewport.width(), m_viewport.height()),
				QBrush(QColor(50, 50, 50)));
			QPen origin(Qt::green, 10, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);


			RendererQtImpl::renderPoint(painter, this, Vector2(0, 0), origin);

			QPen pen(Qt::green, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			if(m_bodyVisible)
			{
				for (auto& body : m_world->bodyList())
				{
					ShapePrimitive primitive;
					primitive.shape = body->shape();
					primitive.rotation = body->angle();
					primitive.transform = body->position();
					RendererQtImpl::renderShape(painter, this, primitive, pen);
				}
			}
			if(m_jointVisible)
			{
				for (auto& joint : m_world->jointList())
					RendererQtImpl::renderJoint(painter, this, joint.get(), pen);
			}
			if(m_axisVisible)
			{
				QColor color = Qt::green;
				color.setAlphaF(0.6);
				pen.setColor(color);
				for (int i = -10; i <= 10; i++)
				{
					RendererQtImpl::renderPoint(painter, this, Vector2(0, i), pen);
					RendererQtImpl::renderPoint(painter, this, Vector2(i, 0), pen);
				}
				color.setAlphaF(0.45);
				pen.setColor(color);
				pen.setWidth(1);
				RendererQtImpl::renderLine(painter, this, Vector2(0, -10), Vector2(0, 10), pen);
				RendererQtImpl::renderLine(painter, this, Vector2(-10, 0), Vector2(10, 0), pen);

			}
			if(m_aabbVisible)
			{
				QPen pen(Qt::cyan, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
				for(auto [body, node]: m_dbvh->leaves())
					RendererQtImpl::renderAABB(painter, this, node->pair.value, pen);
			}
			if(m_dbvhVisible)
			{
				DBVH::Node* root = m_dbvh->root();
				drawDbvh(root, painter);
			}
		}
	}

	bool Camera::aabbVisible() const
	{
		return m_aabbVisible;
	}

	void Camera::setAabbVisible(bool aabbVisible)
	{
		m_aabbVisible = aabbVisible;
	}

	bool Camera::jointVisible() const
	{
		return m_jointVisible;
	}

	void Camera::setJointVisible(bool jointVisible)
	{
		m_jointVisible = jointVisible;
	}

	bool Camera::bodyVisible() const
	{
		return m_bodyVisible;
	}

	void Camera::setBodyVisible(bool bodyVisible)
	{
		m_bodyVisible = bodyVisible;
	}

	bool Camera::axisVisible() const
	{
		return m_axisVisible;
	}

	void Camera::setAxisVisible(bool axisVisible)
	{
		m_axisVisible = axisVisible;
	}

	real Camera::meterToPixel() const
	{
		return m_meterToPixel;
	}

	void Camera::setMeterToPixel(const real& meterToPixel)
	{
		if (meterToPixel < 1.0)
		{
			m_meterToPixel = 1.0;
			m_pixelToMeter = 1.0;
			return;
		}
		m_meterToPixel = meterToPixel;
		m_pixelToMeter = 1.0 / meterToPixel;
	}

	Vector2 Camera::transform() const
	{
		return m_transform;
	}

	void Camera::setTransform(const Vector2& transform)
	{
		m_transform = transform;
	}

	void Camera::setWorld(World* world)
	{
		m_world = world;
	}

	World* Camera::world() const
	{
		return m_world;
	}

	Body* Camera::targetBody() const
	{
		return m_targetBody;
	}

	void Camera::setTargetBody(Body* targetBody)
	{
		m_targetBody = targetBody;
	}

	real Camera::zoomFactor() const
	{
		return m_zoomFactor;
	}

	void Camera::setZoomFactor(const real& zoomFactor)
	{
		m_zoomFactor = zoomFactor;
	}

	bool Camera::dbvhVisible() const
	{
		return m_dbvhVisible;
	}

	void Camera::setDbvhVisible(bool dbvhVisible)
	{
		m_dbvhVisible = dbvhVisible;
    }

    Camera::Viewport Camera::viewport() const
    {
        return m_viewport;
    }

    void Camera::setViewport(const Viewport &viewport)
    {
        m_viewport = viewport;
		m_origin.set((m_viewport.topLeft.x + m_viewport.bottomRight.x) * (0.5), (m_viewport.topLeft.y + m_viewport.bottomRight.y) * (0.5));
    }
	Vector2 Camera::worldToScreen(const Vector2& pos)
	{
		Vector2 real_origin(m_origin.x + m_transform.x, m_origin.y - m_transform.y);
		return Vector2(real_origin.x + pos.x * m_meterToPixel, real_origin.y - pos.y * m_meterToPixel);
	}
	Vector2 Camera::screenToWorld(const Vector2& pos)
	{
		Vector2 real_origin(m_origin.x + m_transform.x, m_origin.y - m_transform.y);
		Vector2 result = pos - real_origin;
		result.y = -result.y;
		result *= m_pixelToMeter;
		return result;
	}
	DBVH* Camera::dbvh() const
	{
		return m_dbvh;
	}

	void Camera::setDbvh(DBVH* dbvh)
	{
		m_dbvh = dbvh;
	}

	bool Camera::visible() const
	{
		return m_visible;
	}

	void Camera::setVisible(bool visible)
	{
		m_visible = visible;
	}

	void Camera::drawDbvh(DBVH::Node* node, QPainter* painter)
	{
		if (node == nullptr)
			return;

		drawDbvh(node->left, painter);
		drawDbvh(node->right, painter);

		QPen pen(Qt::cyan, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		if (!node->isLeaf())
			RendererQtImpl::renderAABB(painter, this, node->pair.value, pen);
	}
	real Camera::Viewport::width()
	{
		return bottomRight.x - topLeft.x;
	}
	real Camera::Viewport::height()
	{
		return bottomRight.y - topLeft.y;
	}
	void Camera::Viewport::setWidth(const real& width)
	{
		bottomRight.x = topLeft.x + width;
	}
	void Camera::Viewport::setHeight(const real& height)
	{
		bottomRight.y = topLeft.y + height;
	}
	void Camera::Viewport::set(const real& width, const real& height)
	{
		setWidth(width);
		setHeight(height);
	}
}
