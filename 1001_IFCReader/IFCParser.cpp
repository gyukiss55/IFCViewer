// IFCParser.cpp : This file contains the 'IFCParser' class implementation
//


#include <windows.h>

#include <stdio.h>
#include <string>
#include <tchar.h>
#include <iostream>
#include <utility>
#include <map>

#include "IFCParser.h"
#include "face3D.h"
#include "GeomUtils.h"



bool IFCParser::CheckClosedBody (const std::vector<std::vector<DWORD>>& cpPoly, std::vector<std::vector<std::pair<DWORD, bool>>>& closedBodies)
{
    std::map<std::size_t, class Edge> edges;
    std::map<DWORD, std::vector<std::pair<DWORD, DWORD>>> polyToEdgeMap;
    std::map<DWORD, bool> polyToOrient;
    std::vector<std::pair<DWORD, bool>> polyProcess;
    std::map<DWORD, DWORD> usedFaces;

    auto AddCheckEdgesFunc = [this](DWORD cp1, DWORD cp2, DWORD iPoly, std::map<std::size_t, class Edge>& edges)
    {
        std::size_t h1 = HashCode (std::size_t (cp1), std::size_t (cp2));
        if (edges.count (h1) != 0) {
            edges[h1].up++;
            edges[h1].p2 = iPoly;
            return false;
        }
        std::size_t h2 = HashCode (std::size_t (cp2), std::size_t (cp1));
        if (edges.count (h2) == 1) {
            class Edge& e = edges[h2];
            e.p2 = iPoly;
            e.um++;
            if (e.up != 1)
                return false;
        }
        else {
            class Edge e (cp1, cp2, iPoly);
            edges[h1] = e;
        }
        return true;
    };

    bool status = true;
    DWORD iPoly = 0;
    for (const std::vector<DWORD>& cps : cpPoly) {
        DWORD cp0 (0);
        DWORD cp1 (0);
        for (auto cp : cps) {
            DWORD cp2 (cp);
            if (cp0 == 0)
                cp0 = cp;
            else {
                if (!AddCheckEdgesFunc (cp1, cp2, iPoly, edges)) {
                    status = false;
                }
            }

            cp1 = cp;
        }
        if (!AddCheckEdgesFunc (cp1, cp0, iPoly, edges)) {
            status = false;
        }
        iPoly++;
    }

    for (auto it = edges.cbegin (); it != edges.cend (); ++it) {
        if (it->second.up >= 1) {
            polyToEdgeMap[it->second.p1].push_back (std::pair<DWORD, DWORD> (it->second.cp1, it->second.cp2));
        }
        if (it->second.up >= 2 || it->second.um >= 1) {
            polyToEdgeMap[it->second.p2].push_back (std::pair<DWORD, DWORD> (it->second.cp1, it->second.cp2));
        }
        if (it->second.up != 1 && it->second.um != 1)
            status = false;
    }

   
    for (polyProcess.push_back (std::pair<DWORD, bool> (0, true)); polyProcess.size () > 0; ) {
        std::pair < DWORD, bool> processPair = polyProcess.back ();
        polyProcess.pop_back ();
            polyToOrient[processPair.first] = processPair.second;
        std::vector<std::pair<DWORD, DWORD>> cpPairVector = polyToEdgeMap[processPair.first];
        for (std::pair<DWORD, DWORD> cpPair : cpPairVector) {
            std::size_t h1 = HashCode (std::size_t (cpPair.first), std::size_t (cpPair.second));
            class Edge& e = edges[h1];
            if (e.up == 1 && e.p1 != processPair.first && polyToOrient.count (e.p1) == 0)
                polyProcess.push_back (std::pair<DWORD, bool>(e.p1, processPair.second));
            if (e.um == 1 && e.p2 != processPair.first && polyToOrient.count (e.p2) == 0)
                polyProcess.push_back (std::pair<DWORD, bool>(e.p2, processPair.second));
            if (e.up == 2 && e.p2 != processPair.first && polyToOrient.count (e.p2) == 0)
                polyProcess.push_back (std::pair<DWORD, bool>(e.p2, !processPair.second));
            if (e.up == 2 && e.p1 != processPair.first && polyToOrient.count (e.p1) == 0)
                polyProcess.push_back (std::pair<DWORD, bool> (e.p1, !processPair.second));
        }
    }

    DWORD bodyIndex = 0;
    for (int bodyNr = 0; bodyNr < 100; ++bodyNr) {
        bool found = false;
        std::vector<std::pair<DWORD, bool>> bodyFaces;

        for (auto it = edges.cbegin (); it != edges.cend (); ++it) {
            if (it->second.up == 1 && it->second.um == 1) {
                if (usedFaces.count (it->second.p1) == 0) {
                    usedFaces[it->second.p1] = bodyIndex;
                    bodyFaces.push_back (std::pair<DWORD, bool>(it->second.p1, polyToOrient[it->second.p1]));
                    found = true;
                }
                if (usedFaces.count (it->second.p2) == 0) {
                    usedFaces[it->second.p2] = bodyIndex;
                    bodyFaces.push_back (std::pair<DWORD, bool>(it->second.p2, polyToOrient[it->second.p2]));
                    found = true;
                }
            }
        }
        if (found)
            closedBodies.push_back (bodyFaces);
        else
            break;
    }
    return closedBodies.size () > 0;
}


void IFCParser::AppendStepID (std::string& entityStr, DWORD stepID)
{
    entityStr.append (std::string ("#"));
    char buff[30];
    snprintf (buff, sizeof (buff), "%d", stepID);
    entityStr.append (std::string (buff));
}


DWORD IFCParser::GenerateClosedBody (DWORD shapeRep, const std::vector<std::vector<DWORD>>& cpPoly, const std::vector<DWORD>& faces, const std::vector<std::vector<std::pair<DWORD, bool>>>& closedBodies)
{
    std::vector <DWORD> breps;
    auto itCPvector = cpPoly.cbegin ();
    DWORD iFace = 0;
    for (DWORD face : faces) {
        const std::vector<DWORD>& cps = *itCPvector;
        DWORD fob = *referencesMap[face].cbegin ();
        DWORD poly = *referencesMap[fob].cbegin ();

        bool invert = false;
        for (const std::vector<std::pair<DWORD, bool>>& b : closedBodies) {
            for (const std::pair<DWORD, bool>& p : b) {
                if (p.first == iFace)
                    invert = !p.second;
            }
        }
        std::string entityStr;
        AppendStepID (entityStr, poly);
        entityStr.append (std::string ("= IFCPOLYLOOP(("));
        bool first = true;

       if (invert) {
            for (auto it = cps.crbegin (); it != cps.crend () ; it++) {
                if (first)
                    first = false;
                else
                    entityStr.append (std::string (","));
                AppendStepID (entityStr, *it);
            }
        } else {
             for (auto it = cps.cbegin (); it != cps.cend () ; it++) {
                if (first)
                    first = false;
                else
                    entityStr.append (std::string (","));
                AppendStepID (entityStr, *it);
            }
        }

        entityStr.append (std::string ("));\r\n"));
        appendString.push_back (entityStr);
        itCPvector++;
        iFace++;
    }
    for (const std::vector<std::pair<DWORD, bool>>& facesClosedBody : closedBodies) {
        std::string entityStr;
        DWORD closedShellIndex = currentStepIndex++;
        AppendStepID (entityStr, closedShellIndex);
        entityStr.append (std::string ("= IFCCLOSEDSHELL(("));
        bool first = true;
        for (const std::pair<DWORD, bool>& facePair : facesClosedBody) {
            if (first)
                first = false;
            else
                entityStr.append (std::string (","));
            AppendStepID (entityStr, faces[facePair.first]);
        }
        entityStr.append (std::string ("));\r\n"));
        appendString.push_back (entityStr);

        entityStr.clear ();

        DWORD facetedIndex = currentStepIndex++;
        AppendStepID (entityStr, facetedIndex);
        entityStr.append (std::string ("= IFCFACETEDBREP("));
        AppendStepID (entityStr, closedShellIndex);
        entityStr.append (std::string (");\r\n"));
        breps.push_back (facetedIndex);
        appendString.push_back (entityStr);
    }
    if (breps.size () > 0) {
        std::string entityStr;
        //DWORD shapeIndex = currentStepIndex++;
        AppendStepID (entityStr, shapeRep);
        entityStr.append (std::string ("= IFCSHAPEREPRESENTATION("));
        AppendStepID (entityStr, geomSubContextId);
        entityStr.append (std::string (",'Body','Brep',("));
        bool first = true;
        for (DWORD brep : breps) {
            if (first)
                first = false;
            else
                entityStr.append (std::string (","));
            AppendStepID (entityStr, brep);
        }
        entityStr.append (std::string ("));\r\n"));
        appendString.push_back (entityStr);

    }
    return 0;
}


std::size_t IFCParser::GetHashCode (DWORD stepIndex)
{
    DWORD index = entityMap[stepIndex];
    const auto& entity = entityVector[index];
    DWORD p0 = entity.Pos ();
    for (DWORD i = 2; i < entity.Len (); ++i) {
        if (readBuffer[i + p0] == '=') {
            i += 2;
            std::string s;

            s.append (std::string (readBuffer + i + p0, entity.Len () - i));
            return std::hash<std::string>{}(s);
        }
    }
    return 0;
}


bool IFCParser::IsEntityType (DWORD stepIndex, const std::string& typeName)
{
    DWORD index = entityMap[stepIndex];
    const auto& entity = entityVector[index];
    DWORD p0 = entity.Pos ();
    for (DWORD i = 2; i < entity.Len (); ++i) {
        if (readBuffer[i + p0] == '=') {
            i += 2;
            for (DWORD j = 0; j < typeName.size (); ++j) {
                if (typeName[j] != readBuffer[i + j + p0]) {
                    return false;
                }
            }
            return true;
        }
    }
    return false;
}


DWORD IFCParser::GetEntitiesByName (const std::string& name, std::vector<DWORD>& entities)
{
    for (const auto& entity : entityVector) {
        DWORD p0 = entity.Pos ();
        for (DWORD i = 2; i < entity.Len (); ++i) {
            if (readBuffer[i + p0] == '=') {
                i += 2;
                bool found = true;
                for (DWORD j = 0; j < name.size (); ++j) {
                    if (name[j] != readBuffer[i + j + p0]) {
                        found = false;
                        break;
                    }
                }
                if (found) {
                    entities.push_back (entity.Index ());
                }
            }
        }
    }
    return (DWORD)entities.size ();
}


DWORD IFCParser::AddReferences (DWORD index)
{
    if (index >= entityVector.size ())
        return 0;
    std::vector<DWORD> references;
    DWORD p0 = entityVector[index].Pos ();
    DWORD l = entityVector[index].Len ();
    DWORD stepIndex = entityVector[index].Index ();
    for (DWORD p = 2; p < l - 1; ++p) {
        if (readBuffer[p + p0] == '=') {
            for (p = p + 2; p < l - 1; ++p) {
                if (readBuffer[p + p0] == '#') {
                    DWORD i = 0;
                    for (p = p + 1; readBuffer[p + p0] >= '0' && readBuffer[p + p0] <= '9'; ++p) {
                        i = 10 * i + readBuffer[p + p0] - '0';
                    }
                    if (i > 0)
                        references.push_back (i);
                    for (; p < l - 1; ++p) {
                        if (readBuffer[p + p0 + 1] == '#')
                            break;
                    }
                }
            }
            break;
        }
    }
    referencesMap[stepIndex] = references;
    return (DWORD)entityVector.size ();
}


DWORD IFCParser::AddEntities ()
{
    for (DWORD p = 0; p < size - 3; ++p) {
        if (readBuffer[p] == '\r' && readBuffer[p + 1] == '\n' && readBuffer[p + 2] == '#') {
            for (DWORD p1 = p + 3; p1 < size - 2; ++p1) {
                if (readBuffer[p1] == '\r' && readBuffer[p1 + 1] == '\n') {
                    DWORD i = 0;
                    for (DWORD p2 = p + 3; readBuffer[p2] >= '0' && readBuffer[p2] <= '9'; ++p2) {
                        i = 10 * i + readBuffer[p2] - '0';
                    }
                    if (currentStepIndex <= i)
                        currentStepIndex = i + 1;
                    entityVector.push_back (IFCEnity (p + 2, p1 - 2 - p, i));
                    entityMap[i] = (DWORD)entityVector.size () - 1;
                    AddReferences ((DWORD)(entityVector.size () - 1));
                    p = p1 - 1;
                    break;
                }
            }
        }
    }

    // build inverz entity vector
    for (auto itMap = referencesMap.cbegin (); itMap != referencesMap.cend (); ++itMap) {
        DWORD stepIndex = itMap->first;
        std::vector<DWORD> vRefereces = itMap->second;

        for (auto referenceIndex : vRefereces) {
            std::vector<DWORD> v = inverzMap[referenceIndex];
            v.push_back (stepIndex);
            inverzMap[referenceIndex] = v;
        }
    }
    return (DWORD)entityVector.size ();
}


DWORD IFCParser::Convert2BREP (const std::vector<DWORD>& shapeReps)
{
    DWORD num = 0;
    for (DWORD shapeRep : shapeReps) {
        // process one shapeRep

        std::vector<DWORD> faces;
        std::vector<std::vector<DWORD>> cpPoly;
        std::vector<DWORD> polyLoops;
        std::map<std::size_t, DWORD> uniqueCP;

        if (referencesMap[shapeRep].size () > 5) {
            for (DWORD shellOrGeom : referencesMap[shapeRep]) {
                if (this->geomSubContextId == 0 && IsEntityType (shellOrGeom, std::string ("IFCGEOMETRICREPRESENTATIONSUBCONTEXT"))) {
                    this->geomSubContextId = shellOrGeom;
                }
                if (IsEntityType (shellOrGeom, std::string ("IFCSHELLBASEDSURFACEMODEL"))) {
                    for (DWORD openShell : referencesMap[shellOrGeom]) {
                        if (IsEntityType (openShell, std::string ("IFCOPENSHELL"))) {
                            for (DWORD face : referencesMap[openShell]) {
                                if (IsEntityType (face, std::string ("IFCFACE"))) {
                                    faces.push_back (face);
                                    for (DWORD outer : referencesMap[face]) {
                                        if (IsEntityType (outer, std::string ("IFCFACEOUTERBOUND"))) {
                                            for (DWORD poly : referencesMap[outer]) {
                                                if (IsEntityType (poly, std::string ("IFCPOLYLOOP"))) {
                                                    deleteEntityMap[poly] = 1;
                                                    polyLoops.push_back (poly);
                                                    std::vector<DWORD> cpVector;
                                                    for (DWORD cp : referencesMap[poly]) {
                                                        std::size_t hachCp = GetHashCode (cp);
                                                        if (uniqueCP.count (hachCp) == 0) {
                                                            uniqueCP[hachCp] = cp;
                                                        }
                                                        else {
                                                            cp = uniqueCP[hachCp];
                                                        }
                                                        cpVector.push_back (cp);
                                                    }
                                                    cpPoly.push_back (cpVector);
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        std::vector<std::vector<std::pair<DWORD, bool>>> closedBodies;
        if (CheckClosedBody (cpPoly, closedBodies)) {
            GenerateClosedBody (shapeRep, cpPoly, faces, closedBodies);
        }
    }

    return num;
}


DWORD IFCParser::WriteHeader (HANDLE hFile)
{
    DWORD dwBytesWritten = 0;
    BOOL bErrorFlag = FALSE;

    const IFCEnity& e = *entityVector.cbegin ();
    bErrorFlag = WriteFile (
        hFile,           // open file handle
        readBuffer,      // start of data to write
        (DWORD)e.Pos (),  // number of bytes to write
        &dwBytesWritten, // number of bytes that were written
        nullptr);            // no overlapped structure

    if (FALSE == bErrorFlag)
    {
    }
    else
    {
    }

    return 0;
}


DWORD IFCParser::WriteOriginalEntities (HANDLE hFile)
{
    DWORD dwBytesWritten = 0;
    BOOL bErrorFlag = FALSE;

    for (const IFCEnity& e : entityVector) {
        if (deleteEntityMap.count (e.Index ()) != 0)
            continue;
        bErrorFlag = WriteFile (
            hFile,           // open file handle
            readBuffer + e.Pos (),      // start of data to write
            (DWORD)e.Len () + 2,  // number of bytes to write
            &dwBytesWritten, // number of bytes that were written
            nullptr);            // no overlapped structure

        if (FALSE == bErrorFlag)
        {
        }
        else
        {
        }


    }
    return 0;
}


DWORD IFCParser::WriteNewEntities (HANDLE hFile)
{
    DWORD dwBytesWritten = 0;
    BOOL bErrorFlag = FALSE;
    for (const auto& s : appendString) {
        bErrorFlag = WriteFile (
            hFile,           // open file handle
            s.c_str (),      // start of data to write
            (DWORD)s.size (),  // number of bytes to write
            &dwBytesWritten, // number of bytes that were written
            nullptr);            // no overlapped structure

        if (FALSE == bErrorFlag)
        {
        }
        else
        {
        }
    }

    return 0;
}


DWORD IFCParser::WriteTail (HANDLE hFile)
{
    DWORD dwBytesWritten = 0;
    BOOL bErrorFlag = FALSE;

    const IFCEnity& e = *entityVector.crbegin ();
    bErrorFlag = WriteFile (
        hFile,           // open file handle
        readBuffer + e.Pos () + e.Len (),      // start of data to write
        (DWORD)(size - e.Pos () - e.Len ()),  // number of bytes to write
        &dwBytesWritten, // number of bytes that were written
        nullptr);            // no overlapped structure

    if (FALSE == bErrorFlag)
    {
    }
    else
    {
    }

    return 0;
}


DWORD IFCParser::WriteConverted (const char* outfilename)
{
    HANDLE hFile;
 
    hFile = CreateFile (outfilename,                // name of the write
        GENERIC_WRITE,          // open for writing
        0,                      // do not share
        NULL,                   // default security
        CREATE_ALWAYS, //CREATE_NEW,             // create new file only
        FILE_ATTRIBUTE_NORMAL,  // normal file
        NULL);                  // no attr. template

    if (hFile == INVALID_HANDLE_VALUE)
    {
        return 1;
    }

    WriteHeader (hFile);
    WriteOriginalEntities (hFile);
    WriteNewEntities (hFile);
    WriteTail (hFile);
    CloseHandle (hFile);
    return 0;
}


DWORD IFCParser::Run (const char* outfilename)
{
    AddEntities ();
    std::vector<DWORD> shapeReps;
    std::vector<DWORD> shapeRepsOfProduct;
    GetEntitiesByName (std::string ("IFCSHAPEREPRESENTATION("), shapeReps);
    for (DWORD shapeRep : shapeReps) {
        std::vector<DWORD> invs = inverzMap[shapeRep];
        for (DWORD inv : invs) {
            if (IsEntityType (inv, std::string ("IFCPRODUCTDEFINITIONSHAPE"))) {
                shapeRepsOfProduct.push_back (shapeRep);
                deleteEntityMap[shapeRep] = 1;
            }
        }
    }
    Convert2BREP (shapeRepsOfProduct);
    WriteConverted (outfilename);

    int err = 0;
    return err;
}


DWORD IFCParser::Run (Faces3D& faces, std::vector<std::pair<DWORD, DWORD>>* faceProductPairs /*= nullptr*/)
{
    AddEntities ();
    std::vector<DWORD> shapeReps;
    std::vector<DWORD> shapeRepsOfProduct;
    GetEntitiesByName (std::string ("IFCSHAPEREPRESENTATION("), shapeReps);
    for (DWORD shapeRep : shapeReps) {
        std::vector<DWORD> invs = inverzMap[shapeRep];
        for (DWORD inv : invs) {
            if (IsEntityType (inv, std::string ("IFCPRODUCTDEFINITIONSHAPE"))) {
                shapeRepsOfProduct.push_back (shapeRep);
                deleteEntityMap[shapeRep] = 1;
            }
        }
    }
    GetFaces (shapeRepsOfProduct, faces, faceProductPairs);
    int err = 0;
    return err;
}


bool IFCParser::GetVector3D (DWORD cpInst, DWORD offset, Eigen::Vector3d& vec3)
{
    DWORD index = entityMap[cpInst];
    const auto& entity = entityVector[index];
    DWORD p0 = entity.Pos ();
    for (DWORD i = 2; i < entity.Len (); ++i) {
        if (readBuffer[i + p0] == '(') {
            i += offset;
            float x = 0., y = 0., z = 0.;
            sscanf_s (readBuffer + p0 + i, "%f,%f,%f", &x, &y, &z);
            vec3 = Eigen::Vector3d {x, y, z};
            return true;
        }
    }
    return false;
}


void IFCParser::Transform (const Eigen::MatrixXd& tran, Eigen::Vector3d& vec3)
{
    Eigen::Vector4d v;
    v << vec3[0] , vec3[1] , vec3[2] , 1.0;
    Eigen::Vector4d vr = tran * v;
    vec3[0] = (float)vr.x () * scaling;
    vec3[1] = (float)vr.y () * scaling;
    vec3[2] = (float)vr.z () * scaling;
}


bool IFCParser::HasIFCGUID (DWORD stepIndex)
{
    DWORD index = entityMap[stepIndex];
    const auto& entity = entityVector[index];
    DWORD p0 = entity.Pos ();
    for (DWORD i = 2; i < entity.Len () - 23; ++i) {
        if (readBuffer[i + p0] == '(') {
            if (readBuffer[i + p0 + 1] == '\'' && readBuffer[i + p0 + 24] == '\'') {
                return true;
            }
        }
    }
    return false;
}

DWORD IFCParser::GetProduct (DWORD shapeRep)
{
    for (DWORD inv1 : inverzMap[shapeRep]) {
        if (IsEntityType (inv1, std::string ("IFCPRODUCTDEFINITIONSHAPE"))) {
            for (DWORD inv2 : inverzMap[inv1]) {
                if (HasIFCGUID (inv2)) {
                    return inv2;
                }
            }
        }
    }
    return 0;
}

void IFCParser::GetColorRepItem (DWORD repItem, Eigen::Vector3d& color3)
{
    /*
#153= IFCCOLOURRGB($,0.152590218967,0.84489204242,0.84489204242);
#154= IFCSURFACESTYLERENDERING(#153,0.,IFCNORMALISEDRATIOMEASURE(1.),$,$,$,IFCNORMALISEDRATIOMEASURE(0.),$,.NOTDEFINED.);
#155= IFCSURFACESTYLE('PEN_MATERIAL40',.BOTH.,(#154));
#157= IFCPRESENTATIONSTYLEASSIGNMENT((#155));
#159= IFCSTYLEDITEM(#151,(#157),$);
    */

    for (DWORD inv1 : inverzMap[repItem]) {
        if (IsEntityType (inv1, std::string ("IFCSTYLEDITEM"))) {
            for (DWORD ref2 : referencesMap[inv1]) {
                if (IsEntityType (ref2, std::string ("IFCPRESENTATIONSTYLEASSIGNMENT"))) {
                    for (DWORD ref3 : referencesMap[ref2]) {
                        if (IsEntityType (ref3, std::string ("IFCSURFACESTYLE"))) {
                            for (DWORD ref4 : referencesMap[ref3]) {
                                if (IsEntityType (ref4, std::string ("IFCSURFACESTYLERENDERING"))) {
                                    for (DWORD ref5 : referencesMap[ref4]) {
                                        if (IsEntityType (ref5, std::string ("IFCCOLOURRGB"))) {
                                            GetVector3D (ref5, 3, color3);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void IFCParser::GetColorShapeRep (DWORD shapeRep, Eigen::Vector3d& color3)
{
    for (DWORD ref1 : referencesMap[shapeRep]) {
        return GetColorRepItem (ref1, color3);
    }
}


void printMatrix (const char* str, Eigen::MatrixXd& tran)
{
    printf_s ("%s \n", str);
    printf_s (" %f,%f,%f,%f \n", tran(0,0), tran (0, 1), tran (0, 2), tran (0, 3));
    printf_s (" %f,%f,%f,%f \n", tran (1, 0), tran (1, 1), tran (1, 2), tran (1, 3));
    printf_s (" %f,%f,%f,%f \n", tran (2, 0), tran (2, 1), tran (2, 2), tran (2, 3));
    printf_s (" %f,%f,%f,%f \n", tran (3, 0), tran (3, 1), tran (3, 2), tran (3, 3));
}


int IFCParser::GetMappingTran (DWORD cartesianTransformationOperator3d, Eigen::MatrixXd& tran)
{
    Eigen::MatrixXd tran1 (4, 4);
    tran1 << 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.;
    //tran1 = Eigen::MatrixXd::Identity ();
    tran = tran1;

    if (!IsEntityType (cartesianTransformationOperator3d, std::string ("IFCCARTESIANTRANSFORMATIONOPERATOR3D")))
        return -1;

    int axis = 0;
    Eigen::Vector3d translate;
    Eigen::Vector3d axis1 ({ 1.f, 0.f, 0.f });
    Eigen::Vector3d axis2 ({ 0.f, 1.f, 0.f });
    Eigen::Vector3d axis3 ({ 0.f, 0.f, 1.f });
    for (DWORD ref2 : referencesMap[cartesianTransformationOperator3d]) {
        if (IsEntityType (ref2, std::string ("IFCCARTESIANPOINT"))) {
            GetVector3D (ref2, 2, translate);
        }
        if (IsEntityType (ref2, std::string ("IFCDIRECTION"))) {
            if (axis == 0) {
                GetVector3D (ref2, 2, axis1);
            } else if (axis == 1) {
                GetVector3D (ref2, 2, axis2);
            } else if (axis == 2) {
                GetVector3D (ref2, 2, axis3);
            }
            axis++;

        }
    }
    
    tran1 << (double)axis1[0], (double)axis2[0], (double)axis3[0], (double)translate[0], (double)axis1[1], (double)axis2[1], (double)axis3[1], (double)translate[1], (double)axis1[2], (double)axis2[2], (double)axis3[2], (double)translate[2], 0., 0., 0., 1.;
    tran = tran1;
    printMatrix ("GetMappingTran", tran1);
    return 0;
}


int IFCParser::GetAxis2Placement3D (DWORD axis2Placement3D, Eigen::MatrixXd& tran)
{
    Eigen::MatrixXd tran1 (4, 4);
    tran1 << 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.;
    //tran1 = Eigen::MatrixXd::Identity ();
    tran = tran1;

    if (!IsEntityType (axis2Placement3D, std::string ("IFCAXIS2PLACEMENT3D")))
        return -1;

    int axis = 0;
    Eigen::Vector3d translate;
    Eigen::Vector3d axis1 ({ 1.f, 0.f, 0.f });
    Eigen::Vector3d axis3 ({ 0.f, 0.f, 1.f });
    Eigen::Vector3d axis2 ({ 0.f, 1.f, 0.f });
    for (DWORD ref2 : referencesMap[axis2Placement3D]) {
        if (IsEntityType (ref2, std::string ("IFCCARTESIANPOINT"))) {
            GetVector3D (ref2, 2, translate);
        }
        if (IsEntityType (ref2, std::string ("IFCDIRECTION"))) {
            if (axis == 0) {
                GetVector3D (ref2, 2, axis3);
                axis++;
            }
            else {
                GetVector3D (ref2, 2, axis1);
                axis2[0] = axis3[1] * axis1[2] - axis3[2] * axis1[1];
                axis2[1] = axis3[2] * axis1[0] - axis3[0] * axis1[2];
                axis2[2] = axis3[0] * axis1[1] - axis3[1] * axis1[0];
            }

        }
    }

    tran1 << (double)axis1[0], (double)axis2[0], (double)axis3[0], (double)translate[0], (double)axis1[1], (double)axis2[1], (double)axis3[1], (double)translate[1], (double)axis1[2], (double)axis2[2], (double)axis3[2], (double)translate[2], 0., 0., 0., 1.;
    tran = tran1;
    //printMatrix ("Tran1", tran1);
    return 0;
}


bool IFCParser::GetMappedShapeRep (DWORD shapeRep, DWORD& mappedShapeRep)
{
    mappedShapeRep = 0;
    for (DWORD ref1 : referencesMap[shapeRep]) {
        if (IsEntityType (ref1, std::string ("IFCMAPPEDITEM"))) {
            for (DWORD ref2 : referencesMap[ref1]) {
                if (IsEntityType (ref2, std::string ("IFCREPRESENTATIONMAP"))) {
                    for (DWORD ref3 : referencesMap[ref2]) {
                        if (IsEntityType (ref3, std::string ("IFCSHAPEREPRESENTATION"))) {
                            mappedShapeRep = ref3;
                            return true;
                        }
                    }
                }
            }
        }
    }
    return false;
}


bool IFCParser::GetMappedTran (DWORD shapeRep, Eigen::MatrixXd& tran)
{
    Eigen::MatrixXd tran1 (4, 4);
    tran1 << 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.;
    //tran1 = Eigen::MatrixX4d::Identity ();
    tran = tran1;
    for (DWORD ref1 : referencesMap[shapeRep]) {
        if (IsEntityType (ref1, std::string ("IFCMAPPEDITEM"))) {
            for (DWORD ref2 : referencesMap[ref1]) {
                if (IsEntityType (ref2, std::string ("IFCCARTESIANTRANSFORMATIONOPERATOR3D"))) {
                    GetMappingTran (ref2, tran1);
                }
            }

            for (DWORD ref2 : referencesMap[ref1]) {
                if (IsEntityType (ref2, std::string ("IFCREPRESENTATIONMAP"))) {
                    for (DWORD ref3 : referencesMap[ref2]) {
                        if (IsEntityType (ref3, std::string ("IFCAXIS2PLACEMENT3D"))) {
                            GetAxis2Placement3D (ref3, tran);
                            Eigen::MatrixXd tranResult (4, 4);
                            tranResult = tran1 * tran;
                            printMatrix ("TranResult", tranResult);
                            tran = tranResult;

                            return true;
                        }
                    }
                }
            }
        }
    }
    return false;
}


DWORD IFCParser::GetGlobalTran (DWORD shapeRep, Eigen::MatrixXd& tran)
{
    GetMappedTran (shapeRep, tran);
    for (DWORD inv1 : inverzMap[shapeRep]) {
        if (IsEntityType (inv1, std::string ("IFCPRODUCTDEFINITIONSHAPE"))) {
            for (DWORD inv2 : inverzMap[inv1]) {
                for (DWORD ref0 : referencesMap[inv2]) {
                    if (IsEntityType (ref0, std::string ("IFCLOCALPLACEMENT"))) {
                        std::list<DWORD> localPlacements;
                        localPlacements.push_back (ref0);
                        while (localPlacements.size () > 0) {
                            DWORD lp = *localPlacements.crbegin ();
                            localPlacements.pop_back ();
                            for (DWORD ref1 : referencesMap[lp]) {
                                if (IsEntityType (ref1, std::string ("IFCAXIS2PLACEMENT3D"))) {
                                    Eigen::MatrixXd tran1 (4, 4);
                                    GetAxis2Placement3D (ref1, tran1);

                                    Eigen::MatrixXd tranResult (4, 4);
                                    tranResult = tran1 * tran;
 //                                   printMatrix ("TranResult", tranResult);
                                    tran = tranResult;

                                } else
                                if (IsEntityType (ref1, std::string ("IFCLOCALPLACEMENT"))) {
                                    localPlacements.push_back (ref1);
                                }
                                
                            }
                        }
                    }
                }
            }
        }
    }
    return 0;
}


DWORD IFCParser::GetFaces (std::vector<DWORD> shapeReps, Faces3D& faces3D, std::vector<std::pair<DWORD, DWORD>>* faceProductPairs/* = nullptr*/)
{
    for (DWORD shapeRep1 : shapeReps) {
        // process one shapeRep

        Eigen::MatrixXd tran (4, 4);
        tran = Eigen::Matrix4d::Identity ();

        GetGlobalTran (shapeRep1, tran);

        static std::vector<DWORD> filter = { 9805, 11811 };

        Faces3D faces3DTmp;
        std::vector<DWORD> faces;
        std::vector<std::vector<DWORD>> cpPoly;
        std::vector<DWORD> polyLoops;
        std::map<std::size_t, std::pair<DWORD, Eigen::Vector3d>> uniqueCP;

        static unsigned long bodyIndex = 0;
        bool closedShell = false;

        if (faceProductPairs != nullptr) {
            DWORD productRef = GetProduct (shapeRep1);
            if (productRef != 0) {
                bool filtered = true;
                for (auto filterItem : filter) {
                    if (filterItem == productRef)
                        filtered = false;
                }
                if (filter.size () == 0)
                    filtered = false;
                if (filtered)
                    continue;

                faceProductPairs->push_back (std::pair<DWORD, DWORD> ((DWORD)faces3D.faces.size (), productRef));
            }
        }
        DWORD shapeRep = 0;
        if (!GetMappedShapeRep (shapeRep1, shapeRep))
            shapeRep = shapeRep1;

        if (referencesMap[shapeRep].size () > 0) {
            for (DWORD shellOrGeom : referencesMap[shapeRep]) {
                if (this->geomSubContextId == 0 && IsEntityType (shellOrGeom, std::string ("IFCGEOMETRICREPRESENTATIONSUBCONTEXT"))) {
                    this->geomSubContextId = shellOrGeom;
                }
                if (IsEntityType (shellOrGeom, std::string ("IFCSHELLBASEDSURFACEMODEL")) || IsEntityType (shellOrGeom, std::string ("IFCFACETEDBREP"))) {
                    Eigen::Vector3d colorBody (0.,1.,0.);
                    GetColorRepItem (shellOrGeom, colorBody);
                    for (DWORD openShell : referencesMap[shellOrGeom]) {
                        if (IsEntityType (openShell, std::string ("IFCOPENSHELL")) || IsEntityType (openShell, std::string ("IFCCLOSEDSHELL"))) {
                            if (IsEntityType (openShell, std::string ("IFCCLOSEDSHELL"))) {
                                ++bodyIndex;
                                closedShell = true;
                            }
                            else {
                                closedShell = false;
                            }
                            for (DWORD face : referencesMap[openShell]) {
                                if (IsEntityType (face, std::string ("IFCFACE"))) {
                                    faces.push_back (face);
                                    for (DWORD bound : referencesMap[face]) {
                                        bool isOuter = IsEntityType (bound, std::string ("IFCFACEOUTERBOUND"));
                                        if (isOuter || IsEntityType (bound, std::string ("IFCFACEBOUND"))) {
                                            std::vector<Eigen::Vector3d> coords;  
                                            for (DWORD poly : referencesMap[bound]) {


                                                if (IsEntityType (poly, std::string ("IFCPOLYLOOP"))) {
                                                    printf_s ("IFCPOLYLOOP:#%d\n", poly);
                                                    deleteEntityMap[poly] = 1;
                                                    polyLoops.push_back (poly);

                                                    std::vector<DWORD> cpVector;
                                                    for (DWORD cp : referencesMap[poly]) {
                                                        std::size_t hachCp = GetHashCode (cp);
                                                        if (uniqueCP.count (hachCp) == 0) {
                                                            Eigen::Vector3d  vec3;
                                                            GetVector3D (cp, 2, vec3);
                                                            Transform (tran, vec3);
                                                            std::pair<DWORD, Eigen::Vector3d> p (cp, vec3);
                                                            uniqueCP[hachCp] = p;
                                                            coords.push_back (vec3);
                                                        }
                                                        else {
                                                            cp = uniqueCP[hachCp].first;
                                                            Eigen::Vector3d  vec3 = uniqueCP[hachCp].second;
                                                            coords.push_back (vec3);
                                                        }
                                                        cpVector.push_back (cp);
                                                    }
                                                    cpPoly.push_back (cpVector);
                                                }
                                            }
                                            {

                                                if (isOuter) {
                                                    Polygon3D* facePtr = new Polygon3D ();
                                                    facePtr->color = RedColor3D;
                                                    facePtr->outerLoop = coords;
                                                    if (closedShell)
                                                        facePtr->bodyIndex = bodyIndex;
                                                    else
                                                        facePtr->bodyIndex = 0;
                                                    facePtr->color = colorBody;
                                                    faces3DTmp.faces.push_back (facePtr);
                                                } else {
                                                    if (faces3DTmp.faces.size () > 0) {
                                                        if (faces3DTmp.faces[faces3DTmp.faces.size () - 1]->type == Face3D::Polygon3DType) {
                                                            static_cast<Polygon3D*>(faces3DTmp.faces[faces3DTmp.faces.size () - 1])->innerLoops.push_back (coords);
                                                        }
                                                    }

                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        for (auto face : faces3DTmp.faces) {
            if (face->type == Face3D::Polygon3DType) {
                Polygon3D* poly = static_cast<Polygon3D*>(face);
                if (poly->innerLoops.size () > 0) {
                    GeomUtils::RemoveHoles (poly);
                }
                
                if (poly->innerLoops.size () == 0) {
                    if (poly->outerLoop.size () == 3) {
                        Triangle3D* facePtr = new Triangle3D ();
                        facePtr->color = poly->color;
                        facePtr->coords[0] = poly->outerLoop[0];
                        facePtr->coords[1] = poly->outerLoop[1];
                        facePtr->coords[2] = poly->outerLoop[2];
                        facePtr->bodyIndex = poly->bodyIndex;
                        facePtr->color = poly->color;
                        faces3D.faces.push_back (facePtr);
                    }
                    else {
                        std::vector<std::array<Eigen::Vector3d, 3>> triangles;
                        GeomUtils::Triangulate (poly->outerLoop, triangles);
                        for (auto triangle : triangles) {
                            Triangle3D* facePtr = new Triangle3D ();
                            facePtr->color = RedColor3D;
                            facePtr->coords[0] = triangle[0];
                            facePtr->coords[1] = triangle[1];
                            facePtr->coords[2] = triangle[2];
                            facePtr->bodyIndex = poly->bodyIndex;
                            facePtr->color = poly->color;
                            faces3D.faces.push_back (facePtr);
                        }
                    }
                }
            }
        }
   }
   return 0;
}



DWORD g_BytesTransferred = 0;


VOID CALLBACK FileIOCompletionRoutine (
    __in  DWORD dwErrorCode,
    __in  DWORD dwNumberOfBytesTransfered,
    __in  LPOVERLAPPED lpOverlapped
);


VOID CALLBACK FileIOCompletionRoutine (
    __in  DWORD dwErrorCode,
    __in  DWORD dwNumberOfBytesTransfered,
    __in  LPOVERLAPPED lpOverlapped)
{
    _tprintf (TEXT ("Error code:\t%x\n"), dwErrorCode);
    _tprintf (TEXT ("Number of bytes:\t%x\n"), dwNumberOfBytesTransfered);
    g_BytesTransferred = dwNumberOfBytesTransfered;
}


int ReadIFCFile (const char* infilename, DWORD& size, char*& readBuffer)
{
    int err = 0;
    HANDLE hFile;
    DWORD  dwBytesRead = 0;
    OVERLAPPED ol = { 0 };
    readBuffer = nullptr;
    size = 0;

    hFile = CreateFile (infilename,               // file to open
        GENERIC_READ,          // open for reading
        FILE_SHARE_READ,       // share for reading
        NULL,                  // default security
        OPEN_EXISTING,         // existing file only
        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, // normal file
        NULL);                 // no attr. template

    if (hFile == INVALID_HANDLE_VALUE)
    {
        return -1;
    }
    DWORD higeSize = 0;
    size = GetFileSize (hFile, &higeSize);
    if (higeSize > 0 || size < 100) {
        std::cout << "Error: file size!\n";
        CloseHandle (hFile);
        return -1;
    }
    readBuffer = new char[size + 1];
    if (readBuffer == nullptr) {
        std::cout << "Error: buffer allocation!\n";
        CloseHandle (hFile);
        return -1;
    }
    memset (readBuffer, 0, size + 1);
    DWORD retSize = 0;
    BOOL status = ReadFileEx (hFile, readBuffer, size, &ol, FileIOCompletionRoutine);
    if (!status) {
        std::cout << "Error: readfile !\n";
        CloseHandle (hFile);
        return -1;

    }

    SleepEx (500, TRUE);
    DWORD dwError = GetLastError ();

    if (g_BytesTransferred != size) {
        std::cout << "Error: readfile !\n";
        CloseHandle (hFile);
        return -1;

    }
    readBuffer[size] = 0;
    CloseHandle (hFile);

    return 0;
}


int IFC2BREP (const char * filename, const char * outfilename)
{
    char* readBuffer = nullptr;
    DWORD size (0);
    ReadIFCFile (filename, size, readBuffer);

    Faces3D faces;
    IFCParser parser (readBuffer, size);
    int err = parser.Run (faces);

    delete[]readBuffer;

    return err;
}



int ReadIFCFaces (const char* filename, Faces3D& faces3D, std::vector<std::pair<DWORD, DWORD>>* faceProductPairs /*= nullptr*/) {
    char* readBuffer = nullptr;
    DWORD size (0);
    ReadIFCFile (filename, size, readBuffer);
    IFCParser parser (readBuffer, size);
    int err = parser.Run (faces3D, faceProductPairs);

    delete[]readBuffer;

    return err;
}
