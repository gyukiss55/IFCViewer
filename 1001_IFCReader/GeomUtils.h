// GeomUtils.h : This file contains the 'GeomUtils' class definition. 
//
#pragma once

#include	<map>
#include	"face3D.h"

namespace GeomUtils {
	int CalculateVolume (const Faces3D& faces, std::map<int, double>& bodyVolumeMap);
	int Triangulate (const std::vector<Eigen::Vector3d>& polyCoords, std::vector<std::array<Eigen::Vector3d, 3>>& triangles);

};