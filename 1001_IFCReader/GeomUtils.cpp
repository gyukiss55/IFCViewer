
#include <igl/opengl/glfw/Viewer.h>
#include "Face3D.h"
#include "GeomUtils.h"
#include "LocalPlace.h"
#include "IFCViewer.h"

namespace GeomUtils {

double CalculateVolume (const std::vector<Eigen::Vector3d>& polygon)
{
	Eigen::Vector3d p0 (polygon[0]);
	Eigen::Vector3d p2 (polygon[1]);
	Eigen::Vector3d s (p0 + p2);
	Eigen::Vector3d n (0.,0.,0.);
	for (unsigned int i = 2; i < polygon.size (); ++i) {
		Eigen::Vector3d p1 (p2);
		p2 = polygon[i];
		Eigen::Vector3d v2 (p2 - p0);
		Eigen::Vector3d v1 (p1 - p0);
		Eigen::Vector3d n1;
		n1 = v1.cross (v2);
		n1 = n1  * .5;
		Eigen::Vector3d n2 ((p1 - p0).cross (p2 - p0) * .5);
		Eigen::Vector3d n0 (n1 - n2);
		n1.cwiseEqual (n2);
		n0.mean ();
		n = n + n1;
		s = s + p2;
	}
	return ((s.z () * n.z ()) / double (polygon.size ()));
}

int CalculateVolume (const Faces3D& faces3D, std::map<int, double>& bodyVolumeMap)
{
	for (const Face3D * face : faces3D.faces) {
		if (face->bodyIndex == 0)
			continue;
		std::vector<Eigen::Vector3d> polygon;
		switch (face->type) {
			case Face3D::Triangle3DType:
				{
					const Triangle3D* poly = static_cast<const Triangle3D*>(face);
					for (auto v3D : poly->coords)
						polygon.push_back (v3D);
				}
				break;
			case Face3D::Quad3DType:
				{
					const Quad3D* poly = static_cast<const Quad3D*>(face);
					for (const Eigen::Vector3d& v3D : poly->coords)
						polygon.push_back (v3D);
				}
				break;
			case Face3D::Polygon3DType:
				{
					const Polygon3D* poly = static_cast<const Polygon3D*>(face);
					for (const Eigen::Vector3d& v3D : poly->outerLoop)
						polygon.push_back (v3D);
				}
				break;
			default:
				break;
		}
		double v = CalculateVolume (polygon);
		if (v != 0.) {
			if (bodyVolumeMap.find (face->bodyIndex) != bodyVolumeMap.cend ())
				bodyVolumeMap [face->bodyIndex] = bodyVolumeMap[face->bodyIndex] + v;
			else
				bodyVolumeMap[face->bodyIndex] = v;
		}
	}
	return 0;
}

double CalculateNormal (const std::vector < Eigen::Vector3d>& polyCoords, Eigen::Vector3d& n)
{
	Eigen::Vector3d p0 (polyCoords[0]);
	Eigen::Vector3d p2 (polyCoords[1]);
	UInt32 lastAdd = 0;
	for (UInt32 i = 2; i < polyCoords.size (); ++i) {
		Eigen::Vector3d p1 = p2;
		p2 = polyCoords[i];
		Eigen::Vector3d n1 = (p1 - p0).cross (p2 - p0);
/*
		IFCBaseGeometry::P3Vector tp1 (p1.x (), p1.y (), p1.z ());
		IFCBaseGeometry::P3Vector tp2 (p2.x (), p2.y (), p2.z ());
		IFCBaseGeometry::P3Vector tp0 (p0.x (), p0.y (), p0.z ());
		IFCBaseGeometry::P3Vector tn1 ((tp1 - tp0) ^ (tp2 - tp0));
		IFCBaseGeometry::P3Vector tn11 (n1.x (), n1.y (), n1.z ());
		IFCBaseGeometry::P3Vector td (tn1 - tn11);
		if (td.IsNonZeroLength ())
			printf_s ("Eigen::Vector3d:%f,%f,%f P3Vector:%f,%f,%f\n", n1.x (), n1.y (), n1.z (), tn1.X (), tn1.Y (), tn1.Z ());
*/
		n = n + n1;
	}
	if (n.norm () > Eps)
		n.normalize ();
	return n.mean ();
}

int ConcavPoints (const std::vector < Eigen::Vector3d>& polyCoords, const Eigen::Vector3d& n, std::map<UInt32, UInt32>& concavVector)
{
	UInt32 lastAdd = 0;
	Eigen::Vector3d p1 (polyCoords[polyCoords.size () - 2]);
	Eigen::Vector3d p2 (polyCoords[polyCoords.size () - 1]);
	for (UInt32 i = 0; i < polyCoords.size (); ++i) {
		Eigen::Vector3d p0 = p1;
		p1 = p2;
		p2 = polyCoords[i];
		Eigen::Vector3d n1 = (p1 - p0).cross (p2 - p1);
		if (polyCoords.size () >= 70) {
			printf_s ("CP i:%d.  n:%f,%f,%f n1:%f,%f,%f\n", i, n.x (), n.y (), n.z (), n1.x (), n1.y (), n1.z ());
		}
		if (n1.dot (n) < -SmallEps) {
			if (i == 0)
				lastAdd = polyCoords.size () - 1;
			else
				concavVector[i - 1] = 1;
		}
		
	}
	if (lastAdd > 0)
		concavVector[lastAdd] = 1;
	return 0;
}

void CalcBBox (const std::vector < Eigen::Vector3d>& polyCoords, Eigen::Vector3d& bboxMin, Eigen::Vector3d& bboxMax)
{
	bboxMin = { 0,0,0 };
	if (polyCoords.size () > 0) {
		bboxMin = polyCoords[0];
		bboxMax = polyCoords[0];
	}
	for (auto coord : polyCoords) {

		if (bboxMin.x () > coord.x ()) {
			bboxMin[0] = coord.x ();
		} else
		if (bboxMax.x () < coord.x ()) {
			bboxMax[0] = coord.x ();
		}

		if (bboxMin.y () > coord.y ()) {
			bboxMin[1] = coord.y ();
		} else
		if (bboxMax.y () < coord.y ()) {
			bboxMax[1] = coord.y ();
		}

		if (bboxMin.z () > coord.z ()) {
			bboxMin[2] = coord.z ();
		} else
		if (bboxMax.z () < coord.z ()) {
			bboxMax[2] = coord.z ();
		}
	}
}

int ShiftCoordsAndConcavs (std::vector < Eigen::Vector3d>& polyCoords, std::map<UInt32, UInt32>& concavVector, const Eigen::Vector3d& normal, UInt32 skipNumber)
{
	if (concavVector.size () < 1)
		return 0;
	int concavNum1 = concavVector.size ();
	std::map<UInt32, std::pair<UInt32, UInt32>> gapMap;
	std::pair<Int32, Int32> firstLast (-1, -1);
	UInt32 len = 0;
	for (auto concavPair : concavVector) {
		if (firstLast.first < 0) {
			firstLast.first = concavPair.first;
		} else {
			gapMap[concavPair.first - firstLast.second] = std::pair<UInt32, UInt32> ((UInt32)firstLast.second, (UInt32)concavPair.first);
		}
		firstLast.second = concavPair.first;
	} 
	
	if (concavVector.size () > 0) {
		gapMap[polyCoords.size () - firstLast.second + firstLast.first] = std::pair<UInt32, UInt32> ((UInt32)firstLast.second, (UInt32)firstLast.first);
	}

	int shiftNum = 0;
	UInt32 i = 0;
	for (auto gapIt = gapMap.crbegin (); i <= skipNumber && gapIt != gapMap.crend (); ++i, gapIt++) {
		shiftNum = gapIt->second.first + 1;
		if (shiftNum >= polyCoords.size ())
			shiftNum -= polyCoords.size ();
	}

	if (shiftNum > 0) {
		std::vector<Eigen::Vector3d> polyCoordsTmp;
		for (UInt32 i = 0, j = shiftNum; i < polyCoords.size (); ++i, ++j) {
			if (j >= polyCoords.size ()) {
				j -= polyCoords.size ();
			}
			polyCoordsTmp.push_back (polyCoords[j]);
		}
		concavVector.clear ();
		ConcavPoints (polyCoordsTmp, normal, concavVector);
		polyCoords = polyCoordsTmp;
	}
	return shiftNum;
}

bool HasCrossSection (const std::vector<Eigen::Vector3d>& polyCoords, UInt32 beginIndex, UInt32 endIndex, UInt32 p1Index, UInt32 p2Index, const Eigen::Vector3d& normal, Eigen::Vector3d * crossSection)
{
	IFCBaseGeometry::P3Vector p1 (ToP3Vector (polyCoords[p1Index]));
	IFCBaseGeometry::P3Vector p2 (ToP3Vector (polyCoords[p2Index]));
	IFCBaseGeometry::P3Vector z (ToP3Vector (normal));
	double len = (p2 - p1).Length ();

	IFCBaseGeometry::LocalTran lt (IFCBaseGeometry::LocalPlace (p1, z, (p2 - p1)));
	for (UInt32 i = beginIndex; i < endIndex - 1; ++i) {
		IFCBaseGeometry::P3Vector p11 (ToP3Vector (polyCoords[i]));
		IFCBaseGeometry::P3Vector p12 (ToP3Vector (polyCoords[i + 1]));
		IFCBaseGeometry::P3Vector p11l;
		IFCBaseGeometry::P3Vector p12l;
		lt.GetLocalCoord (p11, &p11l);
		lt.GetLocalCoord (p12, &p12l);
		if (Geometry::IsNearZero (p11l.Y () - p12l.Y ()))
			continue;
		if (p11l.Y () * p12l.Y () < Eps) {
			double cp = (p11l.X () * p12l.Y () - p12l.X () * p11l.Y ()) / (p12l.Y () - p11l.Y ());
			if (Geometry::IsPositive (cp) && Geometry::IsPositive (len - cp - Eps)) {
				IFCBaseGeometry::P3Vector pCross = lt.GlobalCoord (IFCBaseGeometry::P3Vector (cp, 0., 0.));
/* Debug view
				DebugCrossSection (ToEigenVector (p1), ToEigenVector (p2), ToEigenVector (p11), ToEigenVector (p12), ToEigenVector (pCross));
*/
				if (crossSection != nullptr) {
					*crossSection = Eigen::Vector3d (pCross.X (), pCross.Y (), pCross.Z ());
					return true;
				}
			}
		}
	}
	return false;
}

int Triangulate (const std::vector < Eigen::Vector3d>& polyCoords, std::vector<std::array<Eigen::Vector3d, 3>>& triangles)
{
	Eigen::Vector3d normal (0.,0.,0.);
	CalculateNormal (polyCoords, normal);

	std::map<UInt32, UInt32> concavVector;
	ConcavPoints (polyCoords, normal, concavVector);

	if (concavVector.size () == 0) {
		for (UInt32 i = 1; i < polyCoords.size () - 1; ++i) {
			std::array<Eigen::Vector3d, 3> triangle;
			triangle[0] = polyCoords[0];
			triangle[1] = polyCoords[i];
			triangle[2] = polyCoords[i + 1];
			triangles.push_back (triangle);
		}
	}
	else {
		std::vector<Eigen::Vector3d> polyCoordsTmp (polyCoords);
		UInt32 num = polyCoordsTmp.size ();
/* Debug view
		{
			Eigen::Vector3d bboxMin;
			Eigen::Vector3d bboxMax;
			CalcBBox (polyCoords, bboxMin, bboxMax);
			static Eigen::Vector3d offset = {0.,0.,0.};
			offset.z () = bboxMax.z () - bboxMin.z () + offset.z ();
			DebugPolygon (polyCoords, offset);
			DebugPoints (polyCoords, concavVector, offset);
		}
*/
		UInt32 skipNumber = 0;
		printf_s ("1.  polyCoordsTmp num:%d , concavVector: %d\n", polyCoordsTmp.size (), concavVector.size ());
		for (UInt32 ci = 0; ci < num; ++ci) {
			if (concavVector.size () == 0) {
				break;
			} else if (concavVector.size () == 1) {
				std::vector<UInt32> removed;
				UInt32 i0 = concavVector.begin ()->first;
				UInt32 i2 = i0 + 1;
				if (i2 >= polyCoordsTmp.size ())
					i2 = 0;
				for (UInt32 i = 0; i < polyCoordsTmp.size () - 2; ++i) {
					UInt32 i1 = i2;
					i2 = i1 + 1;
					if (i2 >= polyCoordsTmp.size ())
						i2 = 0;
					std::array<Eigen::Vector3d, 3> triangle;
					triangle[0] = polyCoordsTmp[i0];
					triangle[1] = polyCoordsTmp[i1];
					triangle[2] = polyCoordsTmp[i2];
					triangles.push_back (triangle);
					removed.push_back (i1);

				}
				concavVector.erase (concavVector.begin ());
			}
			else {

				ShiftCoordsAndConcavs (polyCoordsTmp, concavVector, normal, skipNumber);

				std::map<UInt32, UInt32> removed;
				UInt32 i00 = polyCoordsTmp.size () - 2;
				UInt32 i0 = polyCoordsTmp.size () - 1;
				UInt32 i2 = 0;

				for (UInt32 i = 0; i < polyCoordsTmp.size () - 2; ++i) {
					UInt32 i1 = i2;
					i2 = i1 + 1;
					bool hasCrossSection = false;
					Eigen::Vector3d crossSection;
					if (!HasCrossSection (polyCoordsTmp, i2 + 1, polyCoordsTmp.size () - 2, i0, i2, normal, &crossSection)) {
						std::array<Eigen::Vector3d, 3> triangle;
						triangle[0] = polyCoordsTmp[i0];
						triangle[1] = polyCoordsTmp[i1];
						triangle[2] = polyCoordsTmp[i2];
						triangles.push_back (triangle);
						removed[i1] = 1;
						skipNumber = 0;
					}
					else if (removed.size () == 0) {
						
						for (UInt32 j = i + 1; j < polyCoordsTmp.size () - 1; ++j) {
							if (concavVector.count (j) > 0)
								continue;
							if (!HasCrossSection (polyCoordsTmp, i2, polyCoordsTmp.size () - 1, j - 1, j + 1, normal, &crossSection)) {
								
								std::array<Eigen::Vector3d, 3> triangle;
								triangle[0] = polyCoordsTmp[j-1];
								triangle[1] = polyCoordsTmp[j];
								triangle[2] = polyCoordsTmp[j+1];
								triangles.push_back (triangle);
								removed[j] = 1;
								skipNumber = 0;
								break;
							} 
						}
						if (removed.size () == 0) 
							hasCrossSection = true;

					}

					Eigen::Vector3d n00 = (polyCoordsTmp[i00] - polyCoordsTmp[i0]).cross (polyCoordsTmp[i1] - polyCoordsTmp[i0]);
					Eigen::Vector3d n01 = (polyCoordsTmp[i00] - polyCoordsTmp[i0]).cross (polyCoordsTmp[i2] - polyCoordsTmp[i0]);
					if (normal.dot (n01) < -SmallEps || concavVector.count (i2) > 0 || hasCrossSection) {
						if (removed.size () == 0) {
							printf_s ("gáz van\n");
							{
								skipNumber++;
								static double z = 0.;
								z += 1.5;
								Eigen::Vector3d offset = { 0.,0.,z };
								DebugPolygon (polyCoordsTmp, offset);
								DebugPoints (polyCoordsTmp, concavVector, offset);
								DebugPoints (crossSection, Eigen::Vector3d (1,0,1),  offset);
							}
						}
						std::vector<Eigen::Vector3d> polyCoordsTmp2;
						for (UInt32 k = 0; k < polyCoordsTmp.size (); ++k) {
							if (removed.count (k) > 0)
								continue;
							polyCoordsTmp2.push_back (polyCoordsTmp[k]);
						}
						polyCoordsTmp = polyCoordsTmp2;
						concavVector.clear ();
						ConcavPoints (polyCoordsTmp, normal, concavVector);
						printf_s (" polyCoordsTmp num:%d , concavVector: %d\n", polyCoordsTmp.size (), concavVector.size ());
						//num = concavVector.size ();
						/*
						{
							static double z = 0.;
							z += 1.;
							Eigen::Vector3d offset = { 0.,0.,z };
							DebugPolygon (polyCoordsTmp, offset);
							DebugPoints (polyCoordsTmp, concavVector, offset);
						}
						*/
						break;
					}
				}
			}
		}
	}
	return 0;
}

void DivideLongEdges (std::vector<Eigen::Vector3d>& polyCoords, UInt32 num)
{
	if (num == 0) {
		return;
	}
	UInt32 i = polyCoords.size () - 1;
	Eigen::Vector3d p1 = polyCoords[i];
	std::map<UInt32, UInt32, std::less<UInt32>> lengthMap;
	for (Eigen::Vector3d& p2 : polyCoords) {
		double len = (ToP3Vector (p1) - ToP3Vector (p2)).Length () * 1000000.;
		p1 = p2;
		lengthMap[(UInt32)len+i] = i;
		i++;
		if (i >= polyCoords.size ())
			i -= polyCoords.size ();
	}
	UInt32 j = 0;
	std::map<UInt32, UInt32> toDivide;
	for (auto it = lengthMap.crbegin (); it != lengthMap.crend (); ++it) {
		toDivide[it->second] = ++j;
		if (j >= 10)
			break;
	}
	std::vector<Eigen::Vector3d> polyCoordsTmp;
	UInt32 div = 2 * num;
	for (UInt32 i = 0; i < polyCoords.size (); ++i) {
		polyCoordsTmp.push_back (polyCoords[i]);
		if (toDivide.count (i) > 0) {
			j = i + 1;
			if (j >= polyCoords.size ())
				j -= polyCoords.size ();
			Eigen::Vector3d delta = (polyCoords[j] - polyCoords[i]) * (1. / (double)div);
			for (UInt32 k = 1; k < div; k++) {
				polyCoordsTmp.push_back (polyCoords[i] + (delta * (double)k));
			}
		}
	}
	polyCoords = polyCoordsTmp;
	/**/
	{
		Eigen::Vector3d normal (0., 0., 0.);
		CalculateNormal (polyCoords, normal);

		std::map<UInt32, UInt32> concavVector;
		ConcavPoints (polyCoords, normal, concavVector);

		static double z = 0.;
		z += 1.;
		Eigen::Vector3d offset = { 0.,0.,z };
		DebugPolygon (polyCoords, offset);
		DebugPoints (polyCoords, concavVector, offset);
	}
	/**/
}

int RemoveHoles (Polygon3D* poly) {
	int n = poly->innerLoops.size ();
	for (int i = 0; i < n; ++i) {
		double d0 = (ToP3Vector (poly->outerLoop[0]) - ToP3Vector (poly->innerLoops[0][0])).Length ();
		OuterInnerLink outerInnerLinkMap (0,0,0);
		for (int j = 0; j < poly->outerLoop.size (); ++j) {
			for (int k = 0; k < poly->innerLoops.size (); ++k) {
				for (int m = 0; m < poly->innerLoops[k].size (); ++m) {
					double d1 = (ToP3Vector (poly->outerLoop[j]) - ToP3Vector (poly->innerLoops[k][m])).Length ();
					if (d1 < d0) {
						outerInnerLinkMap = OuterInnerLink (j,k,m);
						d0 = d1;
					}
				}
			}
		}
		std::vector<Eigen::Vector3d> polyCoordsTmp;
		for (int j = 0; j <= outerInnerLinkMap.outerIndex; ++j) {
			polyCoordsTmp.push_back (poly->outerLoop[j]);
		}
		UInt32 k = outerInnerLinkMap.innerLoop;
		for (int j = outerInnerLinkMap.innerIndex; j < poly->innerLoops[k].size (); ++j) {
			polyCoordsTmp.push_back (poly->innerLoops[k][j]);
		}
		for (int j = 0; j <= outerInnerLinkMap.innerIndex; ++j) {
			polyCoordsTmp.push_back (poly->innerLoops[k][j]);
		}
		for (int j = outerInnerLinkMap.outerIndex; j < poly->outerLoop.size ();  ++j) {
			polyCoordsTmp.push_back (poly->outerLoop[j]);
		}
		poly->outerLoop = polyCoordsTmp;
		poly->innerLoops.erase (poly->innerLoops.begin () + k);
		if (poly->innerLoops.size () == 0)
			break;
	}
	if (poly->innerLoops.size () == 0)
	DivideLongEdges (poly->outerLoop, n);
	return poly->innerLoops.size ();
}

} // namespace GeomUtils 
