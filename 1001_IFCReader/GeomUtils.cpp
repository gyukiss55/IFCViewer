
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
					for (const Eigen::Vector3d& v3D : poly->coords)
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

int ConcavPoints (const std::vector < Eigen::Vector3d>& polyCoords, Eigen::Vector3d& n, std::map<UInt32, UInt32>& concavVector)
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
		UInt32 num = concavVector.size ();
		{
			Eigen::Vector3d bboxMin;
			Eigen::Vector3d bboxMax;
			CalcBBox (polyCoords, bboxMin, bboxMax);
			static Eigen::Vector3d offset = {0.,0.,0.};
			offset.z () = bboxMax.z () - bboxMin.z () + offset.z ();
			DebugPolygon (polyCoords, offset);
			DebugPoints (polyCoords, concavVector, offset);
		}
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
				std::map<UInt32, UInt32> removed;
				auto it1 = concavVector.begin ();
				UInt32 i0 = it1->first;
				for (UInt32 j = 1; j < concavVector.size (); ++j) {
					auto it2 = ++it1;
					if (it2->first - i0 > 1)
						break;
					i0 = it2->first;
					it1 = it2;
				}
				UInt32 i00 = polyCoordsTmp.size () - 1;
				if (i0 > 0)
					i00 = i0 - 1;
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
					removed[i1] = 1;
					Eigen::Vector3d n00 = (polyCoordsTmp[i00] - polyCoordsTmp[i0]).cross (polyCoordsTmp[i1] - polyCoordsTmp[i0]);
					Eigen::Vector3d n01 = (polyCoordsTmp[i00] - polyCoordsTmp[i0]).cross (polyCoordsTmp[i2] - polyCoordsTmp[i0]);
					if (n00.dot (n01) < -Eps || concavVector.count (i2) > 0) {
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
						break;
					}
				}
			}
		}
	}
	return 0;
}

} // namespace GeomUtils 
