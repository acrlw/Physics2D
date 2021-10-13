#ifndef PHYSICS2D_UTILS_CAMERA_H
#define PHYSICS2D_UTILS_CAMERA_H
#include <QPainter>


#include "include/collision/broadphase/dbvh.h"
#include "include/collision/broadphase/tree.h"
#include "include/common/common.h"
#include "include/dynamics/world.h"
#include "include/math/linear/vector2.h"
#include "include/render/renderer.h"
namespace Physics2D::Utils
{
	class Camera
	{
	public:
        struct Viewport
        {
            Viewport() = default;
            Viewport(const Vector2& top_left, const Vector2& bottom_right) : topLeft(top_left), bottomRight(bottom_right){}
            Vector2 topLeft = {0, 0};
            Vector2 bottomRight= {800, 600};
            real width();
            real height();
            void setWidth(const real& width);
            void setHeight(const real& height);
            void set(const real& width, const real& height);
        };
        Camera() = default;
		void render(QPainter* painter);
		
        bool aabbVisible() const;
        void setAabbVisible(bool aabbVisible);

        bool jointVisible() const;
        void setJointVisible(bool jointVisible);

        bool bodyVisible() const;
        void setBodyVisible(bool bodyVisible);

        bool axisVisible() const;
        void setAxisVisible(bool axisVisible);

        bool gridScaleLineVisible()const;
        void setGridScaleLineVisible(bool visible);

        real axisPointCount()const;
        void setAxisPointCount(real count);

        real meterToPixel() const;
        void setMeterToPixel(const real &meterToPixel);

        Vector2 transform() const;
        void setTransform(const Vector2 &transform);

        void setWorld(PhysicsWorld *world);
        PhysicsWorld* world()const;

        Body *targetBody() const;
        void setTargetBody(Body *targetBody);

        real zoomFactor() const;
        void setZoomFactor(const real &zoomFactor);

        bool dbvhVisible() const;
        void setDbvhVisible(bool dbvhVisible);

        Viewport viewport() const;
        void setViewport(const Viewport &viewport);

        Vector2 worldToScreen(const Vector2& pos)const;
        Vector2 screenToWorld(const Vector2& pos)const;
		
        DBVH* dbvh()const;
        void setDbvh(DBVH* dbvh);

        Tree* tree()const;
        void setTree(Tree* tree);

        bool visible()const;
        void setVisible(bool visible);

        bool treeVisible()const;
        void setTreeVisible(bool visible);
        

        real deltaTime()const;
        void setDeltaTime(const real& deltaTime);

        bool rotationLineVisible()const;
        void setRotationLineVisible(bool visible);

        bool centerVisible() const;
        void setCenterVisible(bool visible);

        bool contactVisible() const;
        void setContactVisible(bool visible);

        ContactMaintainer* maintainer()const;
        void setContactMaintainer(ContactMaintainer* maintainer);
		
    private:
        void drawGridScaleLine(QPainter* painter);
        void drawDbvh(DBVH::Node* node, QPainter* painter);
        void drawTree(int nodeIndex, QPainter* painter);
        void drawContacts(QPainter* painter);
        bool m_visible = true;
        bool m_aabbVisible = true;
        bool m_jointVisible = true;
		bool m_bodyVisible = true;
		bool m_axisVisible = true;
        bool m_dbvhVisible = false;
        bool m_treeVisible = false;
        bool m_gridScaleLineVisible = false;
        bool m_rotationLineVisible = false;
        bool m_centerVisible = false;
        bool m_contactVisible = false;

		
		real m_meterToPixel = 50.0f;
		real m_pixelToMeter = 0.02f;

        real m_targetMeterToPixel = 80.0f;
        real m_targetPixelToMeter = 0.02f;
		
        Vector2 m_transform;
        Vector2 m_origin;
        Viewport m_viewport;
        PhysicsWorld *m_world = nullptr;
        Body *m_targetBody = nullptr;
        DBVH* m_dbvh = nullptr;
        Tree* m_tree = nullptr;
        ContactMaintainer* m_maintainer = nullptr;

		real m_zoomFactor = 1.0f;
        real m_restitution = 2.0f;
        real m_deltaTime = 15.0f;
        real m_axisPointCount = 20.0f;
        

	};

	
}
#endif
