/*  This file is part of the Pinocchio automatic rigging library.
    Copyright (C) 2007 Ilya Baran (ibaran@mit.edu)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef INTERSECTOR_H
#define INTERSECTOR_H

#include "mesh.h"
#include "vecutils.h"

class Intersector {
public:
    Intersector() : mesh(NULL) {}
    Intersector(const Pinocchio::Mesh &m, const Vector3 &inDir) : mesh(&m), dir(inDir) { init(); }
    
    std::vector<Vector3> intersect(const Vector3 &pt, std::vector<int> *outIndices = NULL) const;
    const Vector3 &getDir() const { return dir; }
private:
    void init();
    void getIndex(const Vector2 &pt, int &x, int &y) const;
    
    const Pinocchio::Mesh *mesh;
    Vector3 dir;
    Vector3 v1, v2; //basis
    Rect2 bounds; //within the basis
    
    std::vector<Vector2> points;
    std::vector<Vector3> sNormals; //they are scaled for intersection
    std::vector<std::vector<int> > triangles;
};

#endif //INTERSECTOR_H
