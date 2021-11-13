/****************************************************************************/
// Filename:		FreeTran.h
// Description:
// Project:			IWE
// Contact person:	KGy
/****************************************************************************/

#if !defined	(_FREETRAN_H_)
#define			_FREETRAN_H_

#pragma once

#include	<math.h>

#include	"Real.h"
#include	"LocalPlace.h"



//#if !defined (_LOCAL_PLACE_H_)
//#define	_LOCAL_PLACE_H_
//#endif


namespace IFCBaseGeometry {

/****************************************************************************/
/*								FreeTran									*/
/****************************************************************************/

class FreeTran {
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
	FreeTran ();

	FreeTran (const P3Vector&	tr_x0,
			   const P3Vector&	tr_y0,
			   const P3Vector&	tr_z0,
			   const P3Vector&	offs0);

	/// Creates a FreeTran with start point, z axis and x axis
	FreeTran (const P3Vector&	p0,
			   const P3Vector&	zt,
			   const P3Vector&	xt);

	FreeTran (const LocalPlace&	lp);
	FreeTran (const LocalTran&	lt);

	FreeTran (const FreeTran&		lt,		/* eddigi lt-re ulteti ra a tovabbi lp-t! */
			   const LocalPlace&	lp);


	virtual ~FreeTran ();

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

	bool IsCartesianCoordinateSystem () const;
	bool IsRightHanded () const;

	inline bool FreeTranToP3Vectors (P3Vector	*pPos,
									   P3Vector	*pAxiX,
									   P3Vector	*pAxiY,
									   P3Vector	*pAxiZ) const;

	inline bool FreeTranToLocalTran (LocalTran& lt) const;

	inline const P3Vector& GlobalOrigo (void) const;

	FreeTran operator+ (const LocalPlace&	lp) const;		/* eddigi lt-re ulteti ra a tovabbi lp-t! */
	FreeTran operator+ (const LocalTran&	lt) const;
	FreeTran operator+ (const FreeTran&	lt) const;

	FreeTran operator- (const LocalPlace&	lp) const;		/* azt az lt-t adja vissza, amire raultetve ezt az lp-t, kiadodik az aktualis lt! */
	FreeTran operator- (const FreeTran&	lt) const;

	bool operator== (const FreeTran&	ltRight) const;
	bool operator!= (const FreeTran&	ltRight) const;

// modifiers
	FreeTran& operator= (const FreeTran& v);
	FreeTran& operator= (const LocalPlace& lp);
	FreeTran& operator= (const LocalTran& lt);
	inline void Identity (void);
	inline void MultiplyOffset (double val);
	FreeTran ClearOffset () const;

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
	bool LocalPlaceToTran_Imp (const FreeTran	&lp);
	bool LocalPlaceToTran_Imp (const LocalPlace	&lp);

public:
	static const FreeTran	IdentityFreeTran;
};



#if !defined (DEBUG_SMALL)
#undef	SmallFilter
#endif

/****************************************************************************/
/*								FreeTran									*/
/****************************************************************************/

inline bool FreeTran::IsOk (void) const
{
	return b_Ok;
}

inline const P3Vector& FreeTran::GlobalOrigo (void) const
{
	return (offs);
}

inline bool FreeTran::FreeTranToP3Vectors (P3Vector	*pPos,
											  P3Vector	*pAxiX,
											  P3Vector	*pAxiY,
											  P3Vector	*pAxiZ) const
{
	DBASSERT (pPos != nullptr && pAxiX != nullptr && pAxiY != nullptr && pAxiZ != nullptr);
	*pPos = GlobalOrigo ();
	*pAxiX = GlobalVector (P3Vector::CV::X_UNIT);
	*pAxiY = GlobalVector (P3Vector::CV::Y_UNIT);
	*pAxiZ = GlobalVector (P3Vector::CV::Z_UNIT);
	return b_Ok;
}

inline bool FreeTran::FreeTranToLocalTran (LocalTran& lt) const
{
	if (!IsCartesianCoordinateSystem() || !IsRightHanded())
		return false;

	lt = LocalTran(tr_x, tr_y, tr_z, offs);
	return true;
}

inline void FreeTran::Identity (void)
{
	Identity_Imp ();
	b_inv_e = false;
	b_inv_c = true;
	b_Ok	= true;
}

inline void  FreeTran::MultiplyOffset (double val)
{
	offs	= offs  * val;
	inv_o	= inv_o * val;
}


} // IFCBaseGeometry


#endif // _FREETRAN_H_
