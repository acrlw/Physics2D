#ifndef PHYSICS2D_MPR_H
#define PHYSICS2D_MPR_H
#include "gjk.h"
namespace Physics2D
{
	/// <summary>
	/// Minkowski Portal Refinement
	/// </summary>
	class MPR
	{
	public:
        /// <summary>
        /// Discover a portal for next collision test
        /// </summary>
        /// <param name="shapeA"></param>
        /// <param name="shapeB"></param>
        /// <returns>return a vector from center of simplex to origin, an initial simplex</returns>
        static std::tuple<Vector2, Simplex> discover(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
		/// <summary>
		/// Refine portal close to origin
		/// </summary>
		/// <param name="shapeA"></param>
		/// <param name="shapeB"></param>
		/// <param name="source"></param>
		/// <param name="centerToOrigin"></param>
		/// <param name="iteration"></param>
		/// <returns>return if there is a collision, the final simplex</returns>
		static std::tuple<bool, Simplex> refine(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,const Simplex& source, const Vector2& centerToOrigin, const real& iteration = 50);
	};
}
#endif
