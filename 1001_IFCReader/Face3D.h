#pragma once

#include <array>
#include <list>
#include <vector>

typedef std::array<float, 3> Vector3D;

const Vector3D nullVector3D ({ 0.f, 0.f, 0.f });

const Vector3D RedColor3D ({ 1.f, 0.f, 0.f });
const Vector3D BlueColor3D ({ 0.f, 0.f, 1.f });
const Vector3D GreenColor3D ({ 0.f, 1.f, 0.f });
const Vector3D YellowColor3D ({ 1.f, 1.f, 0.f });
const Vector3D PurpleColor3D ({ 1.f, 0.f, 1.f });
const Vector3D CyanColor3D ({ 0.f, 1.f, 1.f });
const Vector3D DRedColor3D ({ 0.5f, 0.f, 0.f });
const Vector3D DBlueColor3D ({ 0.f, 0.f, .5f });
const Vector3D DGreenColor3D ({ 0.f, .5f, 0.f });
const Vector3D DYellowColor3D ({ .5f, .5f, 0.f });
const Vector3D DPurpleColor3D ({ .5f, 0.f, .5f });
const Vector3D DCyanColor3D ({ 0.f, .5f, .5f });

struct Face3D {
	enum Face3DTypeEnum {
		Face3DType,
		Triangle3DType,
		Quad3DType,
		Polygon3DType
	};

	Face3DTypeEnum type;
	Vector3D color;


	Face3D () : type (Face3DType), color (nullVector3D) {}
	Face3D (Face3DTypeEnum t) : type (t), color (nullVector3D) {}
	~Face3D () {}
};


struct Triangle3D : public Face3D {
	std::array <Vector3D, 3> coords;

	Triangle3D () : Face3D (Triangle3DType), coords ( {nullVector3D, nullVector3D, nullVector3D}) {}
};

struct Quad3D : public Face3D {
	std::array <Vector3D, 4> coords;

	Quad3D () : Face3D (Quad3DType), coords({ nullVector3D, nullVector3D, nullVector3D, nullVector3D}) {}
};

struct Polygon3D : public Face3D {
	std::vector <Vector3D> coords;

	Polygon3D () : Face3D (Polygon3DType) {}
};

struct Faces3D {
	std::list<Face3D*> faces;
	Faces3D () {}
	~Faces3D () { for (Face3D* face : faces) { delete face; } }
};