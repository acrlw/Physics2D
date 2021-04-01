#include "include/math/math.h"
#include "fmt/core.h"
#include "include/collision/algorithm/gjk.h"
#include "include/collision/contact.h"
#include "include/math/algorithm/graphics/2d.h"
#include "include/math/integrator.h"
using namespace Physics2D;
namespace fmt {
	template <>
	struct formatter<Physics2D::Vector2> {
		template <typename ParseContext>
		constexpr auto parse(ParseContext& ctx) {
			return ctx.begin();
		}

		template <typename FormatContext>
		auto format(const Physics2D::Vector2& p, FormatContext& ctx) {
			return format_to(ctx.out(), "({:.8f}, {:.8f})", p.x, p.y);
		}
	};

	template <>
	struct formatter<Physics2D::Vector3> {
		template <typename ParseContext>
		constexpr auto parse(ParseContext& ctx) {
			return ctx.begin();
		}

		template <typename FormatContext>
		auto format(const Physics2D::Vector3& p, FormatContext& ctx) {
			return format_to(ctx.out(), "({:.8f}, {:.8f}, {:.8f})", p.x, p.y, p.z);
		}
	};
}
using namespace Physics2D;
void test1()
{
	Ellipse a;
	a.set(20, 16);
	Polygon b;
	b.append({ 0,4 });
	b.append({ -4,2 });
	b.append({ -2,-2 });
	b.append({ 2,-4 });
	b.append({ 4,0 });
	b.append({ 0,4 });
	ShapePrimitive spa, spb;
	spa.shape = &a;
	spa.translation.set(6, -8);
	spa.rotation = 45;

	spb.shape = &b;
	spb.rotation = -45;
	spb.translation.set(6, 8);
	fmt::print("p: {}\n", b.center());

	auto [isCollide, simplex] = GJK::gjk(spa, spb);
	simplex = GJK::epa(spa, spb, simplex);
	ContactInfo info = GJK::dumpInfo(spa, spb, simplex);

	auto result = GraphicsAlgorithm2D::lineSegmentIntersection({ -4,2 }, { -2,3 }, { -2,0 }, { -3,4 });
	if (result.has_value())
		fmt::print("intersection point:{}\n", result.value());
	else
		fmt::print("no intersection\n");
	auto isOnSegment = GraphicsAlgorithm2D::isPointOnSegment({ 0, 4 }, { 4, 0 }, { 1, 3 });
	fmt::print("is on segment:{} \n", isOnSegment);
	auto pl = GraphicsAlgorithm2D::shortestLengthPointOfEllipse(10, 8, { 16, -6 });
	fmt::print("p:{}\n", pl);
}
void test2()
{
	BodyState state;
	state.position.set(0, 20);
	state.acceleration.set(0, 9.8f);
	state.angularVelocity = 20;
	state.angle = 15;
	state.deltaTime = 1.0f / 60.0f;
	state.lastPosition.set(0, 20);
	state.lastDeltaTime = 1.0f / 60.0f;
	state.lastAngle = 15;
	for (size_t i = 0; i < 100; i += 1)
	{
		state.acceleration.set(0, 9.8f);
		state.angularAcceleration = -8;
		//fmt::print("----step{}-----\n", i);
		fmt::print("position:{}\n", state.position);
		//fmt::print("velocity:{}\n", state.velocity);
		//fmt::print("acceleration:{}\n", state.acceleration);
		//fmt::print("angle:{}\n", state.angle);
		//fmt::print("angularVelocity:{}\n", state.angularVelocity);
		//fmt::print("--------------\n");
		state = Verlet::integrateVelocity(state, 1.0f / 60.0f);
	}
}
void test3()
{
	auto [p1, p2] = GraphicsAlgorithm2D::shortestLengthLineSegmentEllipse(10, 8, { 12, -8 }, { 20, 10 });
	fmt::print("p1{} \np2{} \n", p1, p2);

	auto vtr = GraphicsAlgorithm2D::pointToLineSegment({ 0, 5 }, { 25, -5 }, { 5, -1 });
	fmt::print("point:{}\n", vtr);
}
int main(int argc, char* argv[])
{
	test1();
	return 0;

}