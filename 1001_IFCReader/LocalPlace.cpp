/****************************************************************************/
// Filename:		LocalPlace.cpp									
// Description:
// Project:			IWE
// Contact person:	KGy
/****************************************************************************/

#if defined (WINDOWS)
#include "Win32Interface.hpp"
#endif
#include	"LocalPlace.h"
#include	"FreeTran.h"
//#include	"IFC_Base.h"

#include	<math.h>

// Geometry
//#include	"TM.h"



namespace IFCBaseGeometry {


/****************************************************************************/
/*					P3Vector Global Scope Operators							*/
/****************************************************************************/

P3Vector operator^ (const P3Vector&	vLeft,
					const P3Vector&	vRight)
{
	P3Vector v;
	v.X (vLeft.Y () * vRight.Z () - vLeft.Z () * vRight.Y ());
	v.Y (vLeft.Z () * vRight.X () - vLeft.X () * vRight.Z ());
	v.Z (vLeft.X () * vRight.Y () - vLeft.Y () * vRight.X ());
	return v;
}

P3Vector& P3Vector::operator^= (const P3Vector& vRight)
{
	P3Vector vLeft (*this);
	X (vLeft.Y () * vRight.Z () - vLeft.Z () * vRight.Y ());
	Y (vLeft.Z () * vRight.X () - vLeft.X () * vRight.Z ());
	Z (vLeft.X () * vRight.Y () - vLeft.Y () * vRight.X ());
	return *this;
}

/****************************************************************************/
/*								P3Vector									*/
/****************************************************************************/

const P3Vector P3Vector::CV::O_NULL	(0.0, 0.0, 0.0);	// O Null Vector
const P3Vector P3Vector::CV::X_UNIT	(1.0, 0.0, 0.0);	// X Unit Vector
const P3Vector P3Vector::CV::Y_UNIT	(0.0, 1.0, 0.0);	// Y Unit Vector
const P3Vector P3Vector::CV::Z_UNIT	(0.0, 0.0, 1.0);	// Z Unit Vector

const LocalPlace LocalPlace::IdentityLocalPlace;		// identity LocalPlace
const LocalTran LocalTran::IdentityLocalTran;			// identity LocalTran
const FreeTran FreeTran::IdentityFreeTran;

P3Vector::P3Vector (void)
:	x (0.0),
	y (0.0),
	z (0.0)
{
}

P3Vector::P3Vector (double xi,
					double yi,
					double zi /* = 0.0 */)
:	x (xi),
	y (yi),
	z (zi)
{
}

P3Vector::P3Vector (const P3Vector& v)
:	x (v.x),
	y (v.y),
	z (v.z)
{
}

P3Vector::~P3Vector (void)
{
}

P3Vector P3Vector::operator/ (const double& s) const
{
	P3Vector v;
	if (fabs (s) > SmallEps) {
		v.X (x / s);
		v.Y (y / s);
		v.Z (z / s);
	} else {
		if (s < 0.0) {
			v.X (x / (-SmallEps));
			v.Y (y / (-SmallEps));
			v.Z (z / (-SmallEps));
		} else {
			v.X (x / SmallEps);
			v.Y (y / SmallEps);
			v.Z (z / SmallEps);
		}
	}
	return v;
}

P3Vector P3Vector::UnitVector (void) const
{
	double len = this->Length ();
	if (len > SmallEps) {
		return (*this / len);
	} else {
		DBASSERT (false);
		return CV::X_UNIT;	//	X_UNIT_VECTOR
	}
}

P3Vector P3Vector::UnitVector (double eps) const
{
	DBASSERT (eps > 0.0);
	double len = this->Length ();
	if (len > eps && eps > 0.0) {
		return (*this / len);
	} else {
		DBASSERT (false);
		return CV::X_UNIT;	//	X_UNIT_VECTOR
	}
}

P3Vector P3Vector::Rotate2D (const double& angle) const
{
	P3Vector ret;

	ret.Z (Z ());

	ret.X ((X () * cos (angle)) - (Y () * sin (angle)));
	ret.Y ((X () * sin (angle)) + (Y () * cos (angle)));

	return ret;
}

double P3Vector::Angle (const P3Vector&	v) const
{
	P3Vector	dv (v - *this);
	double		a (Length ());
	double		b (v.Length ());
	double		c (dv.Length ());

	double		fi;

	if (a < SmallEps || b < SmallEps) {
		fi = 0.0;
	} else {
		double co = (a * a + b * b - c * c) / (2.0 * a * b);
		if (co > 1.0) {
			co = 1.0;
		}else if (co < -1.0) {
			co = -1.0;
		}
		fi = acos (co);
		if (fi < 0.0) {
			fi = fi + 2.0 * Pi;
		}
	}
	return (fi);
}

double P3Vector::Angle2D (const P3Vector&	v) const
{
	double fi = this->Angle (v);
	P3Vector vc = *this ^ v;
	if (vc.Z () < 0) {
		fi = -fi;
	}
	return (fi);
}

double P3Vector::PlaneAngle (const P3Vector&	v,
							 const P3Vector&	vNormal) const
{
	P3Vector	dv (*this ^ v);
	double		sign (dv * vNormal);
	double		fi = this->Angle (v);
	DBASSERT (fi <= Pi);
	if (sign < 0.0) {
		fi = 2.0 * Pi - fi;
	}

	return (fi);
}

/****************************************************************************/
/*								LocalPlace									*/
/****************************************************************************/
LocalPlace::LocalPlace (void)
:	pt (P3Vector::CV::O_NULL),
	d1 (P3Vector::CV::Z_UNIT),
	d2 (P3Vector::CV::X_UNIT),
	plp (nullptr)
{
}

LocalPlace::LocalPlace (const P3Vector&	p0)
:	pt (p0),
	d1 (P3Vector::CV::Z_UNIT),
	d2 (P3Vector::CV::X_UNIT),
	plp (nullptr)
{
}

LocalPlace::LocalPlace (const P3Vector&	p0,
						const P3Vector&	zt)
:	pt (p0),
	d1 (zt),
	d2 (P3Vector::CV::X_UNIT),
	plp (nullptr)
{
}

LocalPlace::LocalPlace (const P3Vector&	p0,
						const P3Vector&	zt,
						const P3Vector&	xt)
:	pt (p0),
	d1 (zt),
	d2 (xt),
	plp (nullptr)
{
}

LocalPlace::LocalPlace (const P3Vector&		p0,
						const P3Vector&		zt,
						const P3Vector&		xt,
						const LocalPlace&	lp0)
:	pt (p0),
	d1 (zt),
	d2 (xt),
	plp (nullptr)
{
	try {
		plp = new LocalPlace (lp0);
	}
	catch (...) {
		plp = nullptr;
	}
}

LocalPlace::LocalPlace (const LocalPlace& lp)
:	pt (lp.pt),
	d1 (lp.d1),
	d2 (lp.d2),
	plp (nullptr)
{
	if (lp.plp != nullptr) {
		LocalPlace *p1 = lp.plp;
		LocalPlace *p2 = this;
		while  (p1 != nullptr && p2 != nullptr) {
			try {
				p2->plp = new LocalPlace (p1->pt, p1->d1, p1->d2);
			}
			catch (...) {
				p2->plp = nullptr;
			}
			p1 = p1->plp;
			p2 = p2->plp;
			DBASSERT (p2 != nullptr && p2->plp == nullptr);
		}
	}
}

LocalPlace::~LocalPlace ()
{
	if (this->plp != nullptr) {
		LocalPlace	*pAkt;
		LocalPlace	*pNext;
		pAkt		= this->plp;
		while (pAkt != nullptr) {
			pNext		= pAkt->plp;
			pAkt->plp	= nullptr;
			delete	pAkt;
			pAkt = pNext;
		}
	}
	LocalPlace::plp	= nullptr;
}

LocalPlace& LocalPlace::operator= (const LocalPlace& source)
{
	pt = source.pt;
	d1 = source.d1;
	d2 = source.d2;
	plp = nullptr;

	if (source.plp != nullptr) {
		LocalPlace *p1 = source.plp;
		LocalPlace *p2 = this;
		while  (p1 != nullptr && p2 != nullptr) {
			try {
				p2->plp = new LocalPlace (p1->pt, p1->d1, p1->d2);
			}
			catch (...) {
				p2->plp = nullptr;
			}
			p1 = p1->plp;
			p2 = p2->plp;
			DBASSERT (p2 != nullptr && p2->plp == nullptr);
		}
	}

	return *this;
}

bool LocalPlace::operator== (const LocalPlace& other) const
{
	if (pt.IsNotEqual (other.pt) || d1.IsNotEqual (other.d1) ||
		d2.IsNotEqual (other.d2) || ((plp != nullptr) ^ (other.plp != nullptr)))
		return false;
	if (plp != nullptr && other.plp != nullptr)
		return ((*plp) == (*other.plp));
	return true;
}

bool LocalPlace::operator!= (const LocalPlace& other) const
{
	return	!((*this) == (other));
}

void LocalPlace::SetAll (const P3Vector&	p,
						 const P3Vector&	z,
						 const P3Vector&	x)
{
	pt	= p;
	d1	= z;
	d2	= x;
	DBASSERT (plp == nullptr);
	if (plp) {
		delete plp;
	}
	plp	= nullptr;
}

// new_plp must be allocated dynamically!
void	LocalPlace::SetAll (const P3Vector&	p,
							const P3Vector&	z,
							const P3Vector&	x,
							LocalPlace		*new_plp)
{
	pt	= p;
	d1	= z;
	d2	= x;
	DBASSERT (plp == nullptr);
	if (plp) {
		delete plp;
	}
	plp	= new_plp;
}

/****************************************************************************/
/*								LocalTran									*/
/****************************************************************************/
LocalTran::LocalTran ()
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

LocalTran::LocalTran (const P3Vector&	tr_x0,
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

LocalTran::LocalTran (const LocalPlace &lp)
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

LocalTran::LocalTran (const P3Vector&	p0,
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

LocalTran::LocalTran (const LocalTran	&lt,		/* eddigi lt-re ulteti ra a tovabbi lp-t! */
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

LocalTran::~LocalTran ()
{
}

LocalTran LocalTran::operator+ (const LocalPlace&	lp) const		/* eddigi lt-re ulteti ra a tovabbi lp-t! */
{
	LocalTran lt;
	if (lt.LocalPlaceToTran_Imp (lp)) {
		lt.offs = GlobalCoord (lt.offs);
		lt.MultiplyFromLeft_Imp (tr_x, tr_y, tr_z);	/* megvan a tr2 transzformacio! */
	} else {
		lt = *this;
		lt.b_Ok = false;
	}
	return lt;
}

LocalTran LocalTran::operator+ (const LocalTran&	ltParam) const		/* eddigi lt-re ulteti ra a tovabbi lp-t! */
{
	LocalTran	lt;
	lt = ltParam;
	if (lt.b_Ok) {
		lt.offs = GlobalCoord (lt.offs);
		lt.MultiplyFromLeft_Imp (tr_x, tr_y, tr_z);	/* megvan a tr2 transzformacio! */
	} else {
		lt = *this;
	}
	return lt;
}

LocalTran LocalTran::operator- (const LocalPlace&	lp) const		/* azt az lt-t adja vissza, amire raultetve ezt az lp-t, kiadodik az aktualis lt! */
{
	LocalTran lt (lp);
	LocalTran lt_ret;
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

LocalTran LocalTran::operator- (const LocalTran&	ltParam) const		/* azt az lt-t adja vissza, amire raultetve ezt az lp-t, kiadodik az aktualis lt! */
{
	LocalTran lt;
	lt = ltParam;
	LocalTran lt_ret;
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

bool LocalTran::operator== (const LocalTran&	ltRight) const
{
	return	b_Ok == ltRight.b_Ok &&
			offs == ltRight.offs &&
			tr_x == ltRight.tr_x &&
			tr_y == ltRight.tr_y &&
			tr_z == ltRight.tr_z;
}

bool LocalTran::operator!= (const LocalTran&	ltRight) const
{
	return	b_Ok != ltRight.b_Ok ||
			offs != ltRight.offs ||
			tr_x != ltRight.tr_x ||
			tr_y != ltRight.tr_y ||
			tr_z != ltRight.tr_z;
}

LocalTran& LocalTran::operator= (const LocalTran& v)
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

bool LocalTran::GetLocalCoord (const P3Vector&	global,
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

bool LocalTran::GetLocalVector (const P3Vector&	global,
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

P3Vector LocalTran::GlobalCoord (const P3Vector& local) const
{
	P3Vector v (tr_x * local, tr_y * local, tr_z * local);
	return (v + offs);
}

P3Vector LocalTran::GlobalCoord (double	x,
								 double	y,
								 double	z) const
{
	P3Vector local (x, y, z);
	P3Vector v (tr_x * local, tr_y * local, tr_z * local);
	return (v + offs);
}

P3Vector LocalTran::GlobalVector (const P3Vector& local) const
{
	P3Vector v (tr_x * local, tr_y * local, tr_z * local);
	return (v);
}

P3Vector LocalTran::GlobalVector (double	x,
								  double	y,
								  double	z) const
{
	P3Vector local (x, y, z);
	P3Vector v (tr_x * local, tr_y * local, tr_z * local);
	return (v);
}

LocalTran LocalTran::ClearOffset () const
{
	return LocalTran (tr_x, tr_y, tr_z, P3Vector (0,0,0));
}

void LocalTran::Identity_Imp (void)
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

bool LocalTran::CalculateInvMat_Imp (void) const
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

void LocalTran::MultiplyFromLeft_Imp (const P3Vector& xs,	/* Sor vektorok */
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
void LocalTran::MultiplyFromRight_Imp (const P3Vector& xo,	/* Oszlop vektorok */
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

void LocalTran::LocalPlaceToTran_Part_Imp (const LocalPlace&	lp)
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

bool LocalTran::LocalPlaceToTran_Imp (const LocalPlace&	lp)
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
