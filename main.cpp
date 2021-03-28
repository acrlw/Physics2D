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
	
	Polygon b;
	//b.append({ 8, 4 });
	//b.append({ 6, 6 });
	//b.append({ 4, 5 });
	//b.append({ 3, 4 });
	//b.append({ 4, 3 });
	//b.append({ 6, 2 });
	//b.append({ 8, 3 });
	//b.append({ 8, 4 });
	b.append({ 9,6 });
	b.append({ 7,8 });
	b.append({ 5,7 });
	b.append({ 4,6 });
	b.append({ 5,5 });
	b.append({ 7,4 });
	b.append({ 9,5 });
	b.append({ 9,6 });
	ShapePrimitive spa, spb;
	spa.shape = &a;
	spb.shape = &b;
	spb.position = b.center();
	fmt::print("p: {}", b.center());
	auto [isCollide, simplex] = GJK::gjk(spa, spb);
	return 0;
	
}
