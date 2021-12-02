// GeomUtils.h : This file contains the 'GeomUtils' class definition. 
//
#pragma once

#include	<map>
#include	"face3D.h"
#include	"LocalPlace.h"
#include	"FreeTran.h"
#include	"face3D.h"

namespace GeomUtils {
	int CalculateVolume (const Faces3D& faces, std::map<int, double>& bodyVolumeMap);
	int Triangulate (const std::vector<Eigen::Vector3d>& polyCoords, std::vector<std::array<Eigen::Vector3d, 3>>& triangles);
	int RemoveHoles (Polygon3D* poly);
	inline IFCBaseGeometry::P3Vector ToP3Vector (const Eigen::Vector3d& v)
	{
		return IFCBaseGeometry::P3Vector (v.x (), v.y (), v.z ());
	}

	inline Eigen::Vector3d ToEigenVector (const IFCBaseGeometry::P3Vector& v)
	{
		return Eigen::Vector3d (v.X (), v.Y (), v.Z ());
	}

	inline bool ToP3Vectors (const Eigen::MatrixXd& m, IFCBaseGeometry::P3Vector& a1, IFCBaseGeometry::P3Vector& a2, IFCBaseGeometry::P3Vector& a3, IFCBaseGeometry::P3Vector& o)
	{
		if (m.rows () == 4 && m.cols () == 4) {
			a1.X (m (0, 0));
			a1.Y (m (1, 0));
			a1.Z (m (2, 0));
			a2.X (m (0, 1));
			a2.Y (m (1, 1));
			a2.Z (m (2, 1));
			a3.X (m (0, 2));
			a3.Y (m (1, 2));
			a3.Z (m (2, 2));
			o.X (m (0, 3));
			o.Y (m (1, 3));
			o.Z (m (2, 3));
			return true;
		}
		return false;
	}

};