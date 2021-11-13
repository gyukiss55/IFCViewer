/****************************************************************************/
// Filename:		FreeTran.cpp									
// Description:
// Project:			IWE
// Contact person:	KGy
/****************************************************************************/

#if defined (WINDOWS)
#include "Win32Interface.hpp"
#endif
#include	"FreeTran.h"

#include	<math.h>

// Geometry


namespace IFCBaseGeometry {


/****************************************************************************/
/*								FreeTran									*/
/****************************************************************************/
FreeTran::FreeTran ()
	:	tr_x (P3Vector::CV::X_UNIT),
		tr_y (P3Vector::CV::Y_UNIT),
		tr_z (P3Vector::CV::Z_UNIT),
		offs (P3Vector::CV::O_NULL),
		inv_x (P3Vector::CV::X_UNIT),
		inv_y (P3Vector::CV::Y_UNIT),
		inv_z (P3Vector::CV::Z_UNIT),
		inv_o (P3Vector::CV::O_NULL),
		b_inv_e (false),
		b_inv_c (true),
		b_Ok (true)
{
}

FreeTran::FreeTran (const P3Vector&	tr_x0,
					const P3Vector&	tr_y0,
					const P3Vector&	tr_z0,
					const P3Vector&	offs0)
	:	tr_x (tr_x0),
		tr_y (tr_y0),
		tr_z (tr_z0),
		offs (offs0),
		inv_x (P3Vector::CV::X_UNIT),
		inv_y (P3Vector::CV::Y_UNIT),
		inv_z (P3Vector::CV::Z_UNIT),
		inv_o (P3Vector::CV::O_NULL),
		b_inv_e (false),
		b_inv_c (false),
		b_Ok (true)
{
}

FreeTran::FreeTran (const P3Vector&	p0,
					const P3Vector&	zt,
					const P3Vector&	xt)
	:	tr_x (P3Vector::CV::X_UNIT),			/* transformation matrix 1. row */
		tr_y (P3Vector::CV::Y_UNIT),			/* transformation matrix 2. row */
		tr_z (P3Vector::CV::Z_UNIT),			/* transformation matrix 3. row */
		offs (P3Vector::CV::O_NULL),			/* offset */
		inv_x (P3Vector::CV::X_UNIT),			/* inverse matrix 1. row */
		inv_y (P3Vector::CV::Y_UNIT),			/* inverse matrix 2. row */
		inv_z (P3Vector::CV::Z_UNIT),			/* inverse matrix 3. row */
		inv_o (P3Vector::CV::O_NULL),			/* inverse offset */
		b_inv_e (false),						/* Inverse calculation failed. */
		b_inv_c (false),						/* Inverse valid matrix calc'd */
		b_Ok (false)							/* Good transformation matrix */
{
	LocalPlace lp (p0, zt, xt);
	LocalPlaceToTran_Imp (lp);
}

FreeTran::FreeTran (const LocalPlace &lp)
	:	tr_x (P3Vector::CV::X_UNIT),			/* transformation matrix 1. row */
		tr_y (P3Vector::CV::Y_UNIT),			/* transformation matrix 2. row */
		tr_z (P3Vector::CV::Z_UNIT),			/* transformation matrix 3. row */
		offs (P3Vector::CV::O_NULL),			/* offset */
		inv_x (P3Vector::CV::X_UNIT),			/* inverse matrix 1. row */
		inv_y (P3Vector::CV::Y_UNIT),			/* inverse matrix 2. row */
		inv_z (P3Vector::CV::Z_UNIT),			/* inverse matrix 3. row */
		inv_o (P3Vector::CV::O_NULL),			/* inverse offset */
		b_inv_e (false),						/* Inverse calculation failed. */
		b_inv_c (false),						/* Inverse valid matrix calc'd */
		b_Ok (false)							/* Good transformation matrix */
{
	LocalPlaceToTran_Imp (lp);
}

FreeTran::FreeTran (const LocalTran &lt)
	:	tr_x (P3Vector::CV::X_UNIT),			/* transformation matrix 1. row */
		tr_y (P3Vector::CV::Y_UNIT),			/* transformation matrix 2. row */
		tr_z (P3Vector::CV::Z_UNIT),			/* transformation matrix 3. row */
		offs (P3Vector::CV::O_NULL),			/* offset */
		inv_x (P3Vector::CV::X_UNIT),			/* inverse matrix 1. row */
		inv_y (P3Vector::CV::Y_UNIT),			/* inverse matrix 2. row */
		inv_z (P3Vector::CV::Z_UNIT),			/* inverse matrix 3. row */
		inv_o (P3Vector::CV::O_NULL),			/* inverse offset */
		b_inv_e (false),						/* Inverse calculation failed. */
		b_inv_c (false),						/* Inverse valid matrix calc'd */
		b_Ok (false)							/* Good transformation matrix */
{
	*this = lt;
}

FreeTran::FreeTran (const FreeTran		&lt,		/* eddigi lt-re ulteti ra a tovabbi lp-t! */
					const LocalPlace	&lp)
	:	tr_x (P3Vector::CV::X_UNIT),			/* transformation matrix 1. row */
		tr_y (P3Vector::CV::Y_UNIT),			/* transformation matrix 2. row */
		tr_z (P3Vector::CV::Z_UNIT),			/* transformation matrix 3. row */
		offs (P3Vector::CV::O_NULL),			/* offset */
		inv_x (P3Vector::CV::X_UNIT),			/* inverse matrix 1. row */
		inv_y (P3Vector::CV::Y_UNIT),			/* inverse matrix 2. row */
		inv_z (P3Vector::CV::Z_UNIT),			/* inverse matrix 3. row */
		inv_o (P3Vector::CV::O_NULL),			/* inverse offset */
		b_inv_e (false),						/* Inverse calculation failed. */
		b_inv_c (false),						/* Inverse valid matrix calc'd */
		b_Ok (false)							/* Good transformation matrix */
{
	if (LocalPlaceToTran_Imp (lp)) {
		offs = lt.GlobalCoord (offs);
		MultiplyFromLeft_Imp (lt.tr_x, lt.tr_y, lt.tr_z);	/* megvan a tr2 transzformacio! */
	}
}

FreeTran::~FreeTran ()
{
}

FreeTran FreeTran::operator+ (const LocalPlace&	lp) const		/* eddigi lt-re ulteti ra a tovabbi lp-t! */
{
	FreeTran lt;
	if (lt.LocalPlaceToTran_Imp (lp)) {
		lt.offs = GlobalCoord (lt.offs);
		lt.MultiplyFromLeft_Imp (tr_x, tr_y, tr_z);	/* megvan a tr2 transzformacio! */
	} else {
		lt = *this;
		lt.b_Ok = false;
	}
	return lt;
}

FreeTran FreeTran::operator+ (const LocalTran&	ltParam) const		/* eddigi lt-re ulteti ra a tovabbi lp-t! */
{
	FreeTran	lt (ltParam.GlobalOrigo (), ltParam.GlobalVector (P3Vector::CV::Z_UNIT), ltParam.GlobalVector (P3Vector::CV::X_UNIT));
	if (lt.b_Ok) {
		lt.offs = GlobalCoord (lt.offs);
		lt.MultiplyFromLeft_Imp (tr_x, tr_y, tr_z);	/* megvan a tr2 transzformacio! */
	} else {
		lt = *this;
	}
	return lt;
}

FreeTran FreeTran::operator+ (const FreeTran&	ltParam) const		/* eddigi lt-re ulteti ra a tovabbi lp-t! */
{
	FreeTran	lt;
	lt = ltParam;
	if (lt.b_Ok) {
		lt.offs = GlobalCoord (lt.offs);
		lt.MultiplyFromLeft_Imp (tr_x, tr_y, tr_z);	/* megvan a tr2 transzformacio! */
	} else {
		lt = *this;
	}
	return lt;
}

FreeTran FreeTran::operator- (const LocalPlace&	lp) const		/* azt az lt-t adja vissza, amire raultetve ezt az lp-t, kiadodik az aktualis lt! */
{
	FreeTran lt (lp);
	FreeTran lt_ret;
	if (lt.CalculateInvMat_Imp ()) {
		lt_ret.tr_x = lt.inv_x;
		lt_ret.tr_y = lt.inv_y;
		lt_ret.tr_z = lt.inv_z;
		lt_ret.offs = GlobalCoord (lt.inv_o);
		lt_ret.MultiplyFromLeft_Imp (tr_x, tr_y, tr_z);	/* megvan a keresett transzformacio! */
	} else {
		lt_ret = *this;
		lt_ret.b_Ok = false;
	}
	return lt_ret;
}

FreeTran FreeTran::operator- (const FreeTran&	ltParam) const		/* azt az lt-t adja vissza, amire raultetve ezt az lp-t, kiadodik az aktualis lt! */
{
	FreeTran lt;
	lt = ltParam;
	FreeTran lt_ret;
	if (lt.CalculateInvMat_Imp ()) {
		lt_ret.tr_x = lt.inv_x;
		lt_ret.tr_y = lt.inv_y;
		lt_ret.tr_z = lt.inv_z;
		lt_ret.offs = GlobalCoord (lt.inv_o);
		lt_ret.MultiplyFromLeft_Imp (tr_x, tr_y, tr_z);	/* megvan a keresett transzformacio! */
	} else {
		lt_ret = *this;
		lt_ret.b_Ok = false;
	}
	return lt_ret;
}

bool FreeTran::operator== (const FreeTran&	ltRight) const
{
	return	b_Ok == ltRight.b_Ok &&
			offs == ltRight.offs &&
			tr_x == ltRight.tr_x &&
			tr_y == ltRight.tr_y &&
			tr_z == ltRight.tr_z;
}

bool FreeTran::operator!= (const FreeTran&	ltRight) const
{
	return	b_Ok != ltRight.b_Ok ||
			offs != ltRight.offs ||
			tr_x != ltRight.tr_x ||
			tr_y != ltRight.tr_y ||
			tr_z != ltRight.tr_z;
}

FreeTran& FreeTran::operator= (const FreeTran& v)
{
	if (this == &v) {
		return *this;
	}
	b_Ok	= v.b_Ok;
	tr_x	= v.tr_x;	/* transformation matrix 1. row */
	tr_y	= v.tr_y;	/* transformation matrix 2. row */
	tr_z	= v.tr_z;	/* transformation matrix 3. row */
	offs	= v.offs;	/* offset */
	inv_x	= v.inv_x;	/* inverse transformation matrix 1. row */
	inv_y	= v.inv_y;	/* inverse transformation matrix 2. row */
	inv_z	= v.inv_z;	/* inverse transformation matrix 3. row */
	inv_o	= v.inv_o;	/* inverse transformation offset */

	b_inv_e = v.b_inv_e;
	b_inv_c = v.b_inv_c;
	return *this;
}

FreeTran& FreeTran::operator= (const LocalPlace& lp)
{
	FreeTran tmpTran (lp);
	*this = tmpTran;
	return *this;
}

FreeTran& FreeTran::operator= (const LocalTran& lt)
{
	FreeTran tmpTran (lt.GlobalOrigo (), lt.GlobalVector (P3Vector::CV::Z_UNIT), lt.GlobalVector (P3Vector::CV::X_UNIT));
	*this = tmpTran;
	return *this;
}

FreeTran FreeTran::ClearOffset () const
{
	return FreeTran (tr_x, tr_y, tr_z, P3Vector (0,0,0));
}

bool FreeTran::GetLocalCoord (const P3Vector&	global,
							   P3Vector			*pLocal) const
{
	DBASSERT (b_Ok && pLocal != nullptr);
	if (!b_Ok || pLocal == nullptr) {
		return false;
	}
	CalculateInvMat_Imp ();
	P3Vector v (inv_x * global, inv_y * global, inv_z * global);
	*pLocal = v + inv_o;
	return b_inv_c;
}

bool FreeTran::GetLocalVector (const P3Vector&	global,
								P3Vector		*pLocal) const
{
	DBASSERT (b_Ok && pLocal != nullptr);
	if (!b_Ok || pLocal == nullptr) {
		return false;
	}
	CalculateInvMat_Imp ();
	P3Vector v (inv_x * global, inv_y * global, inv_z * global);
	*pLocal = v;
	return b_inv_c;
}

P3Vector FreeTran::GlobalCoord (const P3Vector& local) const
{
	P3Vector v (tr_x * local, tr_y * local, tr_z * local);
	return (v + offs);
}

P3Vector FreeTran::GlobalCoord (double	x,
								 double	y,
								 double	z) const
{
	P3Vector local (x, y, z);
	P3Vector v (tr_x * local, tr_y * local, tr_z * local);
	return (v + offs);
}

P3Vector FreeTran::GlobalVector (const P3Vector& local) const
{
	P3Vector v (tr_x * local, tr_y * local, tr_z * local);
	return (v);
}

P3Vector FreeTran::GlobalVector (double	x,
								  double	y,
								  double	z) const
{
	P3Vector local (x, y, z);
	P3Vector v (tr_x * local, tr_y * local, tr_z * local);
	return (v);
}


bool FreeTran::IsCartesianCoordinateSystem () const
{
	return (tr_x.IsPerpendicular(tr_y) && tr_x.IsPerpendicular(tr_z) && tr_z.IsPerpendicular(tr_y));
}


bool FreeTran::IsRightHanded () const
{
	P3Vector zVector = tr_x ^ tr_y;
	P3Vector yVector = tr_z ^ tr_x;
	P3Vector xVector = tr_y ^ tr_z;
	bool isRightHanded = true;
	isRightHanded &= (Geometry::IsPositive(xVector * tr_x) && xVector.IsParalell(tr_x));
	isRightHanded &= (Geometry::IsPositive(yVector * tr_y) && yVector.IsParalell(tr_y));
	isRightHanded &= (Geometry::IsPositive(zVector * tr_z) && zVector.IsParalell(tr_z));
	return isRightHanded;
}

void FreeTran::Identity_Imp (void)
{
	tr_x = P3Vector::CV::X_UNIT;
	tr_y = P3Vector::CV::Y_UNIT;
	tr_z = P3Vector::CV::Z_UNIT;
	offs = P3Vector::CV::O_NULL;

	inv_x = P3Vector::CV::X_UNIT;
	inv_y = P3Vector::CV::Y_UNIT;
	inv_z = P3Vector::CV::Z_UNIT;
	inv_o = P3Vector::CV::O_NULL;
}

bool FreeTran::CalculateInvMat_Imp (void) const
{
	if (b_inv_c || b_inv_e) {
		return b_inv_c;			// Returns true if calculated, and false if singular.
	}
	//
	int		i;
	double	a [12];
	double	inva [12];
	int		cols [3];
	double	x [3];

	a [0] = tr_x.X ();
	a [1] = tr_x.Y ();
	a [2] = tr_x.Z ();
	a [3] = tr_y.X ();
	a [4] = tr_y.Y ();
	a [5] = tr_y.Z ();
	a [6] = tr_z.X ();
	a [7] = tr_z.Y ();
	a [8] = tr_z.Z ();
	//Just For Lint:
	a [9]  = 0.0;
	a [10] = 0.0;
	a [11] = 0.0;

	inva [0] = 1.0;
	inva [1] = 0.0;
	inva [2] = 0.0;
	inva [3] = 0.0;
	inva [4] = 1.0;
	inva [5] = 0.0;
	inva [6] = 0.0;
	inva [7] = 0.0;
	inva [8] = 1.0;
	//Just For Lint:
	inva [9] = 0.0;
	inva [10] = 0.0;
	inva [11] = 0.0;

	for (i = 0; i < 3; i++) {
		double	p;
		double	ab;
		int		i3;
		int		ii;
		int		k;
		p	= 0.0;
		i3	= i * 3;
		k	= -1;
		ab = fabs (a [i3 + 0]);
		if (ab > 0.0) {
			p = ab;
			k = 0;
		}
		ab = fabs (a [i3 + 1]);
		if (ab > p) {
			p = ab;
			k = 1;
		}
		ab = fabs (a [i3 + 2]);
		if (ab > p) {
			p = ab;
			k = 2;
		}
		if (k < 0) {		// Error!
			b_inv_e = true;
			inv_x = P3Vector::CV::O_NULL;
			inv_y = P3Vector::CV::O_NULL;
			inv_z = P3Vector::CV::O_NULL;
			inv_o = P3Vector::CV::O_NULL;
			return (b_inv_c);
		}
		cols [i] = k;
		p = 1.0 / a [i3 + k];
		a [i3 + 0] *= p;
		a [i3 + 1] *= p;
		a [i3 + 2] *= p;
		inva [i3 + 0] *= p;
		inva [i3 + 1] *= p;
		inva [i3 + 2] *= p;
		for (ii = 0; ii < 9; ii += 3) {	// The values for ii are: 0,3 and 6.
			if (ii != i3) {
				p = a [ii + k];
				if (fabs (p) > 0.0) {
					a [ii + 0] -= p * a [i3 + 0];
					a [ii + 1] -= p * a [i3 + 1];
					a [ii + 2] -= p * a [i3 + 2];
					inva [ii + 0] -= p * inva [i3 + 0];
					inva [ii + 1] -= p * inva [i3 + 1];
					inva [ii + 2] -= p * inva [i3 + 2];
				}
			}
		}
	}
	x [0] = 0.0;
	x [1] = 0.0;
	x [2] = 0.0;
	x [cols [2]] = inva [6];
	x [cols [1]] = ((inva [3] - a [3] * x [0]) - a [4] * x [1]) - a [5] * x [2];
	x [cols [0]] = ((inva [0] - a [0] * x [0]) - a [1] * x [1]) - a [2] * x [2];
	inva [0] = x [0];
	inva [3] = x [1];
	inva [6] = x [2];
	x [0] = 0.0;
	x [1] = 0.0;
	x [2] = 0.0;
	x [cols [2]] = inva [7];
	x [cols [1]] = ((inva [4] - a [3] * x [0]) - a [4] * x [1]) - a [5] * x [2];
	x [cols [0]] = ((inva [1] - a [0] * x [0]) - a [1] * x [1]) - a [2] * x [2];
	inva [1] = x [0];
	inva [4] = x [1];
	inva [7] = x [2];
	x [0] = 0.0;
	x [1] = 0.0;
	x [2] = 0.0;
	x [cols [2]] = inva [8];
	x [cols [1]] = ((inva [5] - a [3] * x [0]) - a [4] * x [1]) - a [5] * x [2];
	x [cols [0]] = ((inva [2] - a [0] * x [0]) - a [1] * x [1]) - a [2] * x [2];
	inva [2] = x [0];
	inva [5] = x [1];
	inva [8] = x [2];
	inv_x.XYZ (inva [0], inva [1], inva [2]);
	inv_y.XYZ (inva [3], inva [4], inva [5]);
	inv_z.XYZ (inva [6], inva [7], inva [8]);
	inv_o.XYZ (-(inv_x * offs), -(inv_y * offs), -(inv_z * offs));
	b_inv_c = true;
	return b_inv_c;
}

void FreeTran::MultiplyFromLeft_Imp (const P3Vector& xs,	/* Sor vektorok */
									  const P3Vector& ys,
									  const P3Vector& zs)
{
	P3Vector xo (tr_x.X (),tr_y.X (),tr_z.X ());/* Oszlop vektorok */
	P3Vector yo (tr_x.Y (),tr_y.Y (),tr_z.Y ());
	P3Vector zo (tr_x.Z (),tr_y.Z (),tr_z.Z ());
							/* Matrix szorzas */
	tr_x.XYZ (xs * xo, xs * yo, xs * zo);
	tr_y.XYZ (ys * xo, ys * yo, ys * zo);
	tr_z.XYZ (zs * xo, zs * yo, zs * zo);

	b_inv_e = false;
	b_inv_c = false;
}

#if defined (NeedToMultiplyFromRight)
void FreeTran::MultiplyFromRight_Imp (const P3Vector& xo,	/* Oszlop vektorok */
									   const P3Vector& yo,
									   const P3Vector& zo)
{
	/* Matrix szorzas */
	tr_x.XYZ (tr_x * xo, tr_x * yo, tr_x * zo);		/* tr-Xsora = tr_x * Am */
	tr_y.XYZ (tr_y * xo, tr_y * yo, tr_y * zo);		/* tr-Ysora = tr_y * Am */
	tr_z.XYZ (tr_z * xo, tr_z * yo, tr_z * zo);		/* tr-Zsora = tr_z * Am */

	b_inv_e = false;
	b_inv_c = false;
}
#endif


void FreeTran::LocalPlaceToTran_Part_Imp (const LocalPlace&	lp)
{
	P3Vector	a;
	P3Vector	xo (P3Vector::CV::X_UNIT);
	P3Vector	yo (P3Vector::CV::Y_UNIT);	/* Oszlop vektorok */
	P3Vector	zo (P3Vector::CV::Z_UNIT);

	a = lp.Z_Axis ();		/* This direction is handled as the exact Z */
	if (a.IsNonZeroLength ()) {
		zo = a.UnitVector ();
	}

	a = zo ^ lp.X_Axis ();
	if (a.IsNonZeroLength ()) {
		yo = a.UnitVector ();
	}

	a = yo ^ zo;
	if (a.IsNonZeroLength ()) {
		xo = a.UnitVector ();
	}

	a = zo ^ xo;
	if (a.IsNonZeroLength ()) {
		yo = a.UnitVector ();
	}

	P3Vector xs (xo.X (),yo.X (),zo.X ());
	P3Vector ys (xo.Y (),yo.Y (),zo.Y ());	/* Sor vektorok */
	P3Vector zs (xo.Z (),yo.Z (),zo.Z ());

	MultiplyFromLeft_Imp (xs, ys, zs);

	offs.XYZ (xs * offs, ys * offs, zs * offs);
}


bool FreeTran::LocalPlaceToTran_Imp (const LocalPlace&	lp)
{
	const Int32	ChainCircularLimit = 10000;
	Int32		circularDog = 0;

	Identity ();	// Clears the error flags!

	const LocalPlace *p = &lp;

	while (p != nullptr && circularDog++ < ChainCircularLimit) {
		LocalPlaceToTran_Part_Imp (*p);	// b_inv_c set to false!
		offs += p->Point ();
		p = p->L_Base ();
	}

	if (circularDog > ChainCircularLimit) {
		Identity_Imp ();
		b_inv_e = true;		// Matrix cannot be inverted
		b_inv_c = false;	// Inverz matrix is not calculated
		b_Ok	= false;	// No valid transformation is available
	}
	DBASSERT (b_Ok);
	return (b_Ok);
}




} // IFCBaseGeometry