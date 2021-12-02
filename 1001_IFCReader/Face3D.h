#pragma once

#include <array>
#include <list>
#include <vector>
#include <igl/opengl/glfw/Viewer.h>

#include "Real.h"

const Eigen::Vector3d nullVector3D ({ 0.f, 0.f, 0.f });

const Eigen::Vector3d RedColor3D ({ 1.f, 0.f, 0.f });
const Eigen::Vector3d BlueColor3D ({ 0.f, 0.f, 1.f });
const Eigen::Vector3d GreenColor3D ({ 0.f, 1.f, 0.f });
const Eigen::Vector3d YellowColor3D ({ 1.f, 1.f, 0.f });
const Eigen::Vector3d PurpleColor3D ({ 1.f, 0.f, 1.f });
const Eigen::Vector3d CyanColor3D ({ 0.f, 1.f, 1.f });
const Eigen::Vector3d DRedColor3D ({ 0.5f, 0.f, 0.f });
const Eigen::Vector3d DBlueColor3D ({ 0.f, 0.f, .5f });
const Eigen::Vector3d DGreenColor3D ({ 0.f, .5f, 0.f });
const Eigen::Vector3d DYellowColor3D ({ .5f, .5f, 0.f });
const Eigen::Vector3d DPurpleColor3D ({ .5f, 0.f, .5f });
const Eigen::Vector3d DCyanColor3D ({ 0.f, .5f, .5f });

struct Face3D {
	enum Face3DTypeEnum {
		Face3DType,
		Triangle3DType,
		Quad3DType,
		Polygon3DType
	};

	Face3DTypeEnum type;
	Eigen::Vector3d color;
	unsigned long bodyIndex;

	Face3D () : type (Face3DType), color (nullVector3D), bodyIndex (0) {}
	Face3D (Face3DTypeEnum t) : type (t), color (nullVector3D), bodyIndex (0) {}
	~Face3D () {}
};


struct Triangle3D : public Face3D {
	std::array <Eigen::Vector3d, 3> coords;

	Triangle3D () : Face3D (Triangle3DType), coords ( {nullVector3D, nullVector3D, nullVector3D}) {}
};

struct Quad3D : public Face3D {
	std::array <Eigen::Vector3d, 4> coords;

	Quad3D () : Face3D (Quad3DType), coords({ nullVector3D, nullVector3D, nullVector3D, nullVector3D}) {}
};

struct Polygon3D : public Face3D {
	std::vector <Eigen::Vector3d> outerLoop;
	std::vector<std::vector <Eigen::Vector3d>> innerLoops;

	Polygon3D () : Face3D (Polygon3DType) {}
};

struct Faces3D {
	std::vector<Face3D*> faces;
	Faces3D () {}
	~Faces3D () { for (Face3D* face : faces) { delete face; } }
	int Delete (UInt32 i)
	{
		if (i < faces.size ()) {
			delete faces[i];
			faces.erase (faces.begin () + i);
			return faces.size ();
		}
		return -1;
	}
};

struct OuterInnerLink {
	UInt32 outerIndex;
	UInt32 innerLoop;
	UInt32 innerIndex;

	OuterInnerLink () : outerIndex (0), innerLoop (0), innerIndex (0) {}
	OuterInnerLink (UInt32 o, UInt32 l, UInt32 i) : outerIndex (o), innerLoop (l), innerIndex (i) {}
};