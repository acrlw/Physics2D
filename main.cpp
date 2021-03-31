#include "include/math/math.h"
#include "fmt/core.h"
#include "include/collision/algorithm/gjk.h"
#include "include/collision/contact.h"
#include "include/math/algorithm/graphics/2d.h"
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
int main(int argc, char* argv[])
{
	Ellipse a;
	a.set(20, 16);
	Polygon b;
	//b.append({ 8, 4 });
	//b.append({ 6, 6 });
	//b.append({ 4, 5 });
	//b.append({ 3, 4 });
	//b.append({ 4, 3 });
	//b.append({ 6, 2 });
	//b.append({ 8, 3 });
	//b.append({ 8, 4 });
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

	//auto [isCollide, simplex] = GJK::gjk(spa, spb);
	//simplex = GJK::epa(spa, spb, simplex);
	//ContactInfo info = GJK::dumpInfo(spa, spb, simplex);
	//
	//auto result = GraphicsAlgorithm2D::lineSegmentIntersection({-4,2}, {-2,3}, {-2,0}, {-3,4});
	//if (result.has_value())
	//	fmt::print("intersection point:{}\n", result.value());
	//else
	//	fmt::print("no intersection\n");
	//auto isOnSegment = GraphicsAlgorithm2D::isPointOnSegment({ 0, 4 }, { 4, 0 }, { 1, 3 });
	//fmt::print("is on segment:{} \n", isOnSegment);
	//auto p = GraphicsAlgorithm2D::originToLineSegment({-5, 0}, {3, 0});
	//fmt::print("p: {}\n", p);
	//p = GraphicsAlgorithm2D::originToLineSegment({0, -3}, {0, 6});
	//fmt::print("p: {}\n", p); 
	//p = GraphicsAlgorithm2D::originToLineSegment({1, 1}, {4, 5});
	//fmt::print("p: {}\n", p);
	//auto p = GraphicsAlgorithm2D::originToLineSegment({ -4, -2 }, { 2, 0 });
	//fmt::print("p: {}\n", p);
	auto p = GraphicsAlgorithm2D::shortestLengthPointOfEllipse(10, 8, { 16, -6 });
	fmt::print("p:{}\n", p.value());
	return 0;

}