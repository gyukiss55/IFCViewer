// IFCParser.h : This file contains the 'IFCParser' class definition. 
//
#pragma once

#include <windows.h>
#include <vector>
#include <map>
#include <string>

#include <igl/opengl/glfw/Viewer.h>

#include "face3D.h"


#define PRIM1 514229
#define PRIM2 433494437

#define DWORD       unsigned long

class IFCEnity {
    DWORD   pos;
    DWORD   len;
    DWORD   index;
public:
    IFCEnity () : pos (0), len (0), index (0) {}
    IFCEnity (DWORD p, DWORD l, DWORD i) : pos (p), len (l), index (i) {}
    DWORD Pos () const { return pos; }
    DWORD Len () const { return len; }
    DWORD Index () const { return index; }
};

class Edge {
public:
    DWORD   cp1;
    DWORD   cp2;
    DWORD   up;
    DWORD   um;
    DWORD   p1;
    DWORD   p2;
    Edge () : cp1 (0), cp2 (0), p1 (0), p2 (0), up (0), um (0) {}
    Edge (DWORD icp1, DWORD icp2, DWORD ip1) : cp1 (icp1), cp2 (icp2), p1 (ip1), p2 (0), up (1), um (0) {}
    bool operator== (const Edge& e) const {
        return this->cp1 == e.cp1 && this->cp2 == e.cp2;
    }
};


class IFCParser {
    const char* readBuffer;
    DWORD size;
    DWORD currentStepIndex;
    DWORD geomSubContextId;
    std::vector<class IFCEnity> entityVector;
    std::map<DWORD, DWORD> entityMap;
    std::map<DWORD, DWORD> deleteEntityMap;
    std::map<DWORD, std::vector<DWORD>> referencesMap;
    std::map<DWORD, std::vector<DWORD>> inverzMap;
    std::vector<std::string> appendString;
    float scaling;
public:
    IFCParser (const char* b, DWORD s) : readBuffer (b), size (s), currentStepIndex (0), geomSubContextId (0), scaling (1.e-4f) {
    }
    DWORD AddEntities ();
    DWORD AddReferences (DWORD index);
    DWORD GetEntitiesByName (const std::string& name, std::vector<DWORD>& entities);
    DWORD Convert2BREP (const std::vector<DWORD>& shapeReps);
    bool IsEntityType (DWORD stepIndex, const std::string& typeName);
    std::size_t GetHashCode (DWORD stepIndex);
    bool CheckClosedBody (const std::vector<std::vector<DWORD>>& cpPoly, std::vector<std::vector<std::pair<DWORD, bool>>>& closedBodies);
    DWORD GenerateClosedBody (DWORD shapeRep, const std::vector<std::vector<DWORD>>& cpPoly, const std::vector<DWORD>& faces, const std::vector<std::vector<std::pair<DWORD, bool>>>& closedBodies);
    std::size_t HashCode (std::size_t v1, std::size_t v2) { return (v1 ^ PRIM1) + ((v2 << 16) ^ PRIM2); }
    void AppendStepID (std::string& entityStr, DWORD stepID);
    DWORD WriteConverted (const char* outfilename);
    DWORD WriteHeader (HANDLE hFile);
    DWORD WriteOriginalEntities (HANDLE hFile);
    DWORD WriteNewEntities (HANDLE hFile);
    DWORD WriteTail (HANDLE hFile);
    DWORD Run (const char* outfilename);
    DWORD Run (Faces3D& faces, std::vector<std::pair<DWORD, DWORD>>* faceProductPairs = nullptr);
    DWORD GetFaces (std::vector<DWORD> shapeReps, Faces3D& faces, std::vector<std::pair<DWORD, DWORD>>* faceProductPairs = nullptr);
    bool GetVector3D (DWORD cpInst, DWORD offset, Eigen::Vector3d& vec3);
    DWORD GetProduct (DWORD shapeRep);
    bool HasIFCGUID (DWORD stepIndex);
    bool GetMappedTran (DWORD shapeRep, Eigen::MatrixXd& tran);
    bool GetMappedShapeRep (DWORD shapeRep, DWORD& mappedShapeRep);
    int GetAxis2Placement3D (DWORD axis2Placement3D, Eigen::MatrixXd& tran);
    DWORD GetGlobalTran (DWORD shapeRep, Eigen::MatrixXd& tran);
    void Transform (const Eigen::MatrixXd& tran, Eigen::Vector3d& vec3);
    void GetColor (DWORD shapeRep, Eigen::Vector3d& color3);
};


int ReadIFCFile (const char* filename, DWORD& size, char*& readBuffer);
int IFC2BREP (const char* infilename, const char* outfilename);
int ReadIFCFaces (const char* filename, Faces3D& faces3D, std::vector<std::pair<DWORD, DWORD>>* faceProductPairs = nullptr);

