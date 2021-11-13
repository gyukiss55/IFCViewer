#pragma once

#define	Int32		long
#define	UInt32		unsigned long

#define	Eps			1.E-5
#define SmallEps	1.E-7
#define	Pi			3.14159265358979323846

#define DBASSERT(a) (a);

namespace Geometry {
	inline bool IsNearZero (double v, double tolerance = Eps) { return (fabs (v) < tolerance); }
	inline bool IsPositive (double v, double tolerance = Eps) { return (v > tolerance); }
	inline bool IsNear (double v1, double v2, double tolerance = Eps) { return (fabs (v1 - v2) < tolerance); }
	inline bool IsNotNear (double v1, double v2, double tolerance = Eps) { return (fabs (v1 - v2) >= tolerance); }
}
