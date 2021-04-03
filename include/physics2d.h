#pragma once
#include "include/math/integrator.h"
#include "include/collision/algorithm/sat.h"
#include "include/collision/algorithm/gjk.h"
#include "include/collision/contact.h"
#include "include/collision/detector.h"
#include "include/collision/collider.h"
#include "include/common/common.h"
#include "include/dynamics/shape.h"
#include "include/dynamics/rigidbody.h"
#include "include/math/algorithm/graphics/2d.h"
#include "include/math/math.h"

#include "fmt/core.h"
#include "fmt/format.h"

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

	template <>
	struct formatter<Physics2D::Matrix2x2> {
		template <typename ParseContext>
		constexpr auto parse(ParseContext& ctx) {
			return ctx.begin();
		}

		template <typename FormatContext>
		auto format(const Physics2D::Matrix2x2& p, FormatContext& ctx) {
			return format_to(ctx.out(), "\n({:.8f}, {:.8f})\n({:.8f}, {:.8f})\n", p.column1.x, p.column2.x, p.column1.y, p.column2.y);
		}
	};
	template <>
	struct formatter<Physics2D::Matrix3x3> {
		template <typename ParseContext>
		constexpr auto parse(ParseContext& ctx) {
			return ctx.begin();
		}

		template <typename FormatContext>
		auto format(const Physics2D::Matrix3x3& p, FormatContext& ctx) {
			return format_to(ctx.out(), "\n({:.8f}, {:.8f}, {:.8f})\n({:.8f}, {:.8f}, {:.8f})\n({:.8f}, {:.8f}, {:.8f})\n", 
			    p.column1.x, p.column2.x, p.column3.x, 
				p.column1.y, p.column2.y, p.column3.y, 
				p.column1.z, p.column2.z, p.column3.z);
		}
	};
}