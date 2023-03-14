#ifndef PHYSICS2D_SIMPLEX_H
#define PHYSICS2D_SIMPLEX_H
#include "minkowski.h"
#include "algorithm_2d.h"
namespace Physics2D
{

	/// <summary>
	/// Simplex structure for gjk/epa test.
	/// By convention:
	///   1 points: p0 , construct a single point
	///   2 points: p0 -> p1, construct a segment
	/// >=4 points: p0 -> p1 -> p2 -> p0, construct a polygon
	///	ATTENTION:
	///	  The performance bottleneck results in Container::Vector. Inserting and reallocating is expensive.
	/// </summary>
	/// <returns></returns>
	struct Simplex
	{
		Container::Vector<Minkowski> vertices;
		bool isContainOrigin = false;
		bool containOrigin(bool strict = false);
		static bool containOrigin(const Simplex& simplex, bool strict = false);

		void insert(const size_t& pos, const Minkowski& vertex);
		bool contains(const Minkowski& minkowski);
		bool fuzzyContains(const Minkowski& minkowski, const real& epsilon = 0.0001);

		Vector2 lastVertex() const;
	};
}
#endif