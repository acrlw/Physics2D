#include "include/math/math.h"
#include "fmt/core.h"
#include "include/collision/algorithm/gjk.h"
#include "include/collision/contact.h"

namespace fmt {
	template <>
	struct formatter<Physics2D::Vector2> {
		template <typename ParseContext>
		constexpr auto parse(ParseContext& ctx) {
			return ctx.begin();
		}

		template <typename FormatContext>
		auto format(const Physics2D::Vector2& p, FormatContext& ctx) {
			return format_to(ctx.out(), "({:.4f}, {:.4f})", p.x, p.y);
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
			return format_to(ctx.out(), "({:.4f}, {:.4f}, {:.4f})", p.x, p.y, p.z);
		}
	};
}
using namespace Physics2D;
int main(int argc, char *argv[])
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
	spa.translation.set(6, -2);
	spa.rotation = 45;
	
	spb.shape = &b;
	spb.rotation = -45;
	spb.translation.set(6, 6);
	fmt::print("p: {}\n", b.center());
	
	auto [isCollide, simplex] = GJK::gjk(spa, spb);
	simplex = GJK::epa(spa, spb, simplex);
	ContactInfo info = GJK::dumpInfo(spa, spb, simplex);
	
	auto result = Physics2D::lineSegmentIntersection({-4,2}, {-2,3}, {-2,0}, {-3,4});
	if (result.has_value())
		fmt::print("intersection point:{}\n", result.value());
	else
		fmt::print("no intersection\n");
	return 0;
	
}
