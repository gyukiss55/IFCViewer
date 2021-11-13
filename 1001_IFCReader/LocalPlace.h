/****************************************************************************/
// Filename:		LocalPlace.h
// Description:
// Project:			IWE
// Contact person:	KGy
/****************************************************************************/

#if !defined	(_LOCALPLACE_H_)
#define			_LOCALPLACE_H_

#pragma once

#include	<math.h>

#include	"Real.h"


namespace IFCBaseGeometry {

#if defined (DEBUG_SMALL)
inline double SmallFilter (double	a)
{
	if (fabs (a) < SmallerEps) {
		a = 0.0;
	}
	return a;
}
#else
#define	SmallFilter(a) (a)
#endif

// Templates
template <typename T> inline bool IsSmallEnough (const T&	a)
{
	return (a < SmallEps && a > -SmallEps);
}

template <typename T> inline bool IsSmallEnough (const T&	a,
												 const T&	b)
{
	return (a < b && a > (- b));
}

template <typename T> inline bool IsNear (const T&	a,
										  const T&	b)
{
	return Geometry::IsNear (a, b);
}

template <typename T> inline bool IsNear (const T&	a,
										  const T&	b,
										  const T&	c)
{
	return (a + c > b && a < b + c);	/* c should be > 0 */
}

template <typename T> inline bool IsNotTooSmall (const T&	a)
{
	return (a <= (-SmallEps) || a >= SmallEps);
}

template <typename T> inline bool IsNotTooSmall (const T&	a,
												 const T&	c)
{
	return (a <= (-c) || a >= c);	/* c should be > 0 */
}

template <typename T> inline bool IsNotNear (const T&	a,
											 const T&	b)
{
	return Geometry::IsNotNear (a, b);
}

template <typename T> inline bool IsNotNear (const T&	a,
											 const T&	b,
											 const T&	c)
{
	return (a + c <= b || a >= b + c);	/* c should be > 0 */
}


/****************************************************************************/
/*						P3Vector free operators								*/
/****************************************************************************/

/****************************************************************************/
/*								P3Vector									*/
/****************************************************************************/

class P3Vector {
	double	x;
	double	y;
	double	z;

public:
// constructors
	P3Vector (void);
	P3Vector (double x0,
			  double y0,
			  double z0 = 0.0);
	P3Vector (const P3Vector& v);

	~P3Vector (void);

// modifiers
	inline void X (double x0);
	inline void Y (double y0);
	inline void Z (double z0);

	inline void XYZ (double x0,
					 double y0,
					 double z0 = 0.0);

	inline void SetToZero (void);

	inline P3Vector& operator= (const P3Vector& v);
	inline P3Vector& operator+= (const P3Vector& v);
	inline P3Vector& operator-= (const P3Vector& v);

	P3Vector& operator^= (const P3Vector& v);

	inline P3Vector& operator*= (double s);

// accessors
	P3Vector operator/ (const double& s) const;
	P3Vector UnitVector (void) const;
	P3Vector UnitVector (double eps) const;

	P3Vector Rotate2D (const double& angle) const;

	double Angle (const P3Vector&	v) const;
	double Angle2D (const P3Vector&	v) const;

	double PlaneAngle (const P3Vector&	v,
					   const P3Vector&	vNormal) const;

	inline double X (void) const;
	inline double Y (void) const;
	inline double Z (void) const;
	inline double Length (void) const;
	inline double TaxiDistance (void) const;
	inline bool IsZeroLength (double	eps = SmallEps) const;
	inline bool IsNonZeroLength (double	eps = SmallEps) const;

	inline bool IsEqual (const P3Vector&	v,
						 double				eps = SmallEps) const;

	inline bool IsNotEqual (const P3Vector&	v,
							double			eps = SmallEps) const;

	inline bool IsParalell		(const P3Vector&	v) const;

	inline bool IsPerpendicular	(const P3Vector&	v) const;

// static
	struct CV {							// Constant Vectors
		static const P3Vector O_NULL;	// O Null Vector
		static const P3Vector X_UNIT;	// X Unit Vector
		static const P3Vector Y_UNIT;	// Y Unit Vector
		static const P3Vector Z_UNIT;	// Z Unit Vector
	};
};

/****************************************************************************/
/*					P3Vector Global Scope Operators							*/
/****************************************************************************/

P3Vector operator^ (const P3Vector&	vLeft,
					const P3Vector&	vRight);

double operator* (const P3Vector&	vLeft,
				  const P3Vector&	vRight);



/****************************************************************************/
/*								P3Vector in line							*/
/****************************************************************************/
inline P3Vector& P3Vector::operator= (const P3Vector& v)
{
	if (this == &v) {
		return *this;
	}
	x = v.x;
	y = v.y;
	z = v.z;
	return *this;
}

inline P3Vector& P3Vector::operator+= (const P3Vector& v)
{
	X (x + v.x);
	Y (y + v.y);
	Z (z + v.z);
	return *this;
}

inline P3Vector& P3Vector::operator-= (const P3Vector& v)
{
	X (x - v.x);
	Y (y - v.y);
	Z (z - v.z);
	return *this;
}

inline P3Vector& P3Vector::operator*= (double s)
{
	X (x * s);
	Y (y * s);
	Z (z * s);
	return *this;
}

//Get

inline double P3Vector::X (void) const
{
	return x;
}

inline double P3Vector::Y (void) const
{
	return y;
}

inline double P3Vector::Z (void) const
{
	return z;
}

inline double P3Vector::Length (void) const
{
	return SmallFilter (sqrt(x*x + y*y + z*z));
}

inline double P3Vector::TaxiDistance (void) const
{
	return SmallFilter (fabs(x) + fabs(y) + fabs(z));
}

inline bool P3Vector::IsZeroLength (double	eps /* = SmallEps */) const
{
	return (fabs(x) + fabs(y) + fabs(z) <= eps);
}

inline bool P3Vector::IsNonZeroLength (double	eps /* = SmallEps */) const
{
	return (fabs(x) + fabs(y) + fabs(z) > eps);
}

inline bool P3Vector::IsEqual (const P3Vector&	v,
							   double			eps /* = SmallEps */) const
{
	return fabs (x - v.x) < eps && fabs (y - v.y) < eps && fabs (z - v.z) < eps;
}

inline bool P3Vector::IsNotEqual (const P3Vector&	v,
								  double			eps /* = SmallEps */) const
{
	return fabs (x - v.x) > eps || fabs (y - v.y) > eps || fabs (z - v.z) > eps;
}


inline bool P3Vector::IsParalell (const P3Vector& v) const
{
	return Geometry::IsNearZero ((*this ^ v).Length ());
}


inline bool P3Vector::IsPerpendicular (const P3Vector& v) const
{
	if (this->IsZeroLength () || v.IsZeroLength ())
		return true;

	if (Geometry::IsNearZero ((*this) * v))
		return true;

	return false;
}


//Set
inline void P3Vector::X (double xi)
{
	x = SmallFilter (xi);
}

inline void P3Vector::Y (double yi)
{
	y = SmallFilter (yi);
}

inline void P3Vector::Z (double zi)
{
	z = SmallFilter (zi);
}

inline void P3Vector::XYZ (double xi,
						   double yi,
						   double zi /* = 0.0 */)
{
	x = SmallFilter (xi);
	y = SmallFilter (yi);
	z = SmallFilter (zi);
}

inline void P3Vector::SetToZero (void)
{
	x = 0.0;
	y = 0.0;
	z = 0.0;
}

inline double operator* (const P3Vector&	vLeft,
						 const P3Vector&	vRight)
{
	double s;
	s = SmallFilter (vLeft.X () * vRight.X () +
					 vLeft.Y () * vRight.Y () +
					 vLeft.Z () * vRight.Z ());
	return s;
}

inline P3Vector operator+ (const P3Vector&	vLeft,
						   const P3Vector&	vRight)
{
	P3Vector s (vLeft.X () + vRight.X (),
				vLeft.Y () + vRight.Y (),
				vLeft.Z () + vRight.Z ());
	return s;
}

inline P3Vector operator- (const P3Vector&	vLeft,
						   const P3Vector&	vRight)
{
	P3Vector s (vLeft.X () - vRight.X (),
				vLeft.Y () - vRight.Y (),
				vLeft.Z () - vRight.Z ());
	return s;
}

inline P3Vector operator* (const double&	vLeft,
						 const P3Vector&	vRight)
{
	P3Vector s (vLeft * vRight.X (), vLeft * vRight.Y (), vLeft * vRight.Z ());
	return s;
}

inline P3Vector operator* (const P3Vector&	vLeft,
						 const double&	vRight)
{
	P3Vector s (vLeft.X () * vRight, vLeft.Y () * vRight, vLeft.Z () * vRight);
	return s;
}

inline bool operator== (const P3Vector&	vLeft,
						const P3Vector& vRight)
{
	return	IsNear (vLeft.X (), vRight.X ()) &&
			IsNear (vLeft.Y (), vRight.Y ()) &&
			IsNear (vLeft.Z (), vRight.Z ());
}

inline bool operator!= (const P3Vector&	vLeft,
						const P3Vector& vRight)
{
	return	IsNotNear (vLeft.X (), vRight.X ()) ||
			IsNotNear (vLeft.Y (), vRight.Y ()) ||
			IsNotNear (vLeft.Z (), vRight.Z ());
}



/****************************************************************************/
/*								LocalPlace									*/
/****************************************************************************/

class LocalPlace {

	P3Vector	pt;		/*  StartPoint */
	P3Vector	d1;		/*  New Z_Axis */
	P3Vector	d2;		/*  New X_Axis */
	LocalPlace	*plp;	/*  Predestor  */

public:
// constructors
	LocalPlace (void);

	LocalPlace (const P3Vector&		p0);

	LocalPlace (const P3Vector&		p0,
				const P3Vector&		zt);

	LocalPlace (const P3Vector&		p0,
				const P3Vector&		zt,
				const P3Vector&		xt);

	LocalPlace (const P3Vector&		p0,
				const P3Vector&		zt,
				const P3Vector&		xt,
				const LocalPlace&	lp0);

	LocalPlace (const LocalPlace&	lp);

	virtual ~LocalPlace ();

	LocalPlace& operator= (const LocalPlace& source);

	bool operator== (const LocalPlace&	other) const;
	bool operator!= (const LocalPlace&	other) const;

// accessors
	inline const P3Vector&	Point  (void) const;
	inline const P3Vector&	Z_Axis (void) const;
	inline const P3Vector&	X_Axis (void) const;
	inline LocalPlace*		L_Base (void) const;

// modifiers
	inline void			Point  (const P3Vector&		new_pt);
	inline void			Z_Axis (const P3Vector&		new_d1);
	inline void			X_Axis (const P3Vector&		new_d2);

	void				SetAll (const P3Vector&		p,
								const P3Vector&		z,
								const P3Vector&		x);

	void				SetAll (const P3Vector&		p,
								const P3Vector&		z,
								const P3Vector&		x,
								LocalPlace			*new_plp);	// new_plp must be allocated dynamically!

public:
	static const LocalPlace	IdentityLocalPlace;
};

/****************************************************************************/
/*								LocalTran									*/
/****************************************************************************/

class LocalTran {
private:
	P3Vector	tr_x;				/* transformation matrix 1. row */
	P3Vector	tr_y;				/* transformation matrix 2. row */
	P3Vector	tr_z;				/* transformation matrix 3. row */
	P3Vector	offs;				/* offset */

	mutable P3Vector	inv_x;		/* inverse matrix 1. row */
	mutable P3Vector	inv_y;		/* inverse matrix 2. row */
	mutable P3Vector	inv_z;		/* inverse matrix 3. row */
	mutable P3Vector	inv_o;		/* inverse offset */

	mutable bool		b_inv_e;	/* Inverse calculation failed. */
	mutable bool		b_inv_c;	/* Inverse valid matrix calc'd */
	bool				b_Ok;		/* Good transformation matrix */

public:
// constructors
	LocalTran ();

	LocalTran (const P3Vector&	tr_x0,
			   const P3Vector&	tr_y0,
			   const P3Vector&	tr_z0,
			   const P3Vector&	offs0);

	/// Creates a LocalTran with start point, z axis and x axis
	LocalTran (const P3Vector&	p0,
			   const P3Vector&	zt,
			   const P3Vector&	xt);

	LocalTran (const LocalPlace&	lp);

	LocalTran (const LocalTran&		lt,		/* eddigi lt-re ulteti ra a tovabbi lp-t! */
			   const LocalPlace&	lp);

	virtual ~LocalTran ();

// accessors
	bool GetLocalCoord (const P3Vector&	global,
					    P3Vector		*pLocal) const;

	bool GetLocalVector (const P3Vector&	global,
						 P3Vector			*pLocal) const;

	P3Vector GlobalCoord (const P3Vector& local) const;

	P3Vector GlobalCoord (double	x,
						  double	y,
						  double	z) const;

	P3Vector GlobalVector (const P3Vector& local) const;

	P3Vector GlobalVector (double	x,
						   double	y,
						   double	z) const;

	inline bool IsOk (void) const;

	inline bool IsIdentityTran (void) const;

	inline bool LocalTranToLocalPlace (P3Vector	*pPos,
									   P3Vector	*pAxi,
									   P3Vector	*pDir) const;

	inline bool LocalTranToLocalPlace (LocalPlace *lp) const;

	inline const P3Vector& GlobalOrigo (void) const;

	LocalTran operator+ (const LocalPlace&	lp) const;		/* eddigi lt-re ulteti ra a tovabbi lp-t! */
	LocalTran operator+ (const LocalTran&	lt) const;

	LocalTran operator- (const LocalPlace&	lp) const;		/* azt az lt-t adja vissza, amire raultetve ezt az lp-t, kiadodik az aktualis lt! */
	LocalTran operator- (const LocalTran&	lt) const;

	bool operator== (const LocalTran&	ltRight) const;
	bool operator!= (const LocalTran&	ltRight) const;

// modifiers
	LocalTran& operator= (const LocalTran& v);
	inline void Identity (void);
	inline bool LocalPlaceToLocalTran (const LocalPlace&	lp);
	inline void MultiplyOffset (double val);
	LocalTran ClearOffset () const;

private:
// implementation
	void Identity_Imp (void);

	bool CalculateInvMat_Imp (void) const;

	void MultiplyFromLeft_Imp (const P3Vector& xs,	/* Sor vektorok */
							   const P3Vector& ys,
							   const P3Vector& zs);

	void MultiplyFromRight_Imp (const P3Vector& xo,	/* Oszlop vektorok */
								const P3Vector& yo,
								const P3Vector& zo);

	void LocalPlaceToTran_Part_Imp (const LocalPlace	&lp);
	bool LocalPlaceToTran_Imp (const LocalPlace	&lp);

public:
	static const LocalTran	IdentityLocalTran;
};


class Mirror {
	P3Vector	v;
	double		d;

	Mirror ();

public:
	Mirror (const P3Vector&		vInit,
			double				dInit)
		:	v (vInit.UnitVector ()),
			d (dInit)
	{
	}
	Mirror (const P3Vector&		vInit,
			const P3Vector&		ptInit)
		:	v (vInit.UnitVector ()),
			d (ptInit * vInit.UnitVector ())
	{
	}
	~Mirror ()
	{
	}

	void Set (const P3Vector&		vInit,
			  double				dInit)
	{
		v = vInit.UnitVector ();
		d = dInit;
	}
	void Set (const P3Vector&		vInit,
			  const P3Vector&		ptInit)
	{
		v = vInit.UnitVector ();
		d = ptInit * v;
	}
	P3Vector MirrorCoord (const P3Vector&	ptIn) const
	{
		return ptIn + (2.0 * ((d - (ptIn * v)) * v));
	}
	P3Vector MirrorVector (const P3Vector&	vIn) const
	{
		return vIn - (2.0 * ((vIn * v) * v));
	}
};


#if !defined (DEBUG_SMALL)
#undef	SmallFilter
#endif

/****************************************************************************/
/*								LocalPlace									*/
/****************************************************************************/

// Set
inline void LocalPlace::Point (const P3Vector&	new_pt)
{
	pt  = new_pt;
}

inline void LocalPlace::Z_Axis (const P3Vector&	new_d1)
{
	d1  = new_d1;
}

inline void LocalPlace::X_Axis (const P3Vector&	new_d2)
{
	d2  = new_d2;
}


// Get
inline const P3Vector&	LocalPlace::Point (void) const
{
	return pt;
}

inline const P3Vector&	LocalPlace::Z_Axis (void) const
{
	return d1;
}

inline const P3Vector&	LocalPlace::X_Axis (void) const
{
	return d2;
}

inline LocalPlace*	LocalPlace::L_Base (void) const
{
	return plp;
}

/****************************************************************************/
/*								LocalTran									*/
/****************************************************************************/

inline bool LocalTran::IsOk (void) const
{
	return b_Ok;
}

inline bool LocalTran::IsIdentityTran (void) const
{
	LocalTran identityTran;

	return *this == identityTran;
}

inline const P3Vector& LocalTran::GlobalOrigo (void) const
{
	return (offs);
}

inline bool LocalTran::LocalTranToLocalPlace (P3Vector	*pPos,
											  P3Vector	*pAxi,
											  P3Vector	*pDir) const
{
	DBASSERT (pPos != nullptr && pAxi != nullptr && pDir != nullptr);
	*pPos = GlobalOrigo ();
	*pAxi = GlobalVector (P3Vector::CV::Z_UNIT);
	*pDir = GlobalVector (P3Vector::CV::X_UNIT);
	return b_Ok;
}

inline bool LocalTran::LocalTranToLocalPlace (LocalPlace	*plp) const
{
	DBASSERT (plp != nullptr);
	plp->SetAll (GlobalOrigo (),
			   GlobalVector (P3Vector::CV::Z_UNIT),
			   GlobalVector (P3Vector::CV::X_UNIT));
	return b_Ok;
}

//Ignores the original LocalTran, and makes another one based on lp!
inline bool LocalTran::LocalPlaceToLocalTran (const LocalPlace&	lp)
{
	return LocalPlaceToTran_Imp (lp);
}

inline void LocalTran::Identity (void)
{
	Identity_Imp ();
	b_inv_e = false;
	b_inv_c = true;
	b_Ok	= true;
}

inline void  LocalTran::MultiplyOffset (double val)
{
	offs	= offs  * val;
	inv_o	= inv_o * val;
}



} // IFCBaseGeometry


#endif
