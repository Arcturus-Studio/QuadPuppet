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

#include <fstream>
#include <sstream>
#include <set>

#include "attachment.h"
#include "vecutils.h"
#include "lsqSolver.h"

using namespace Pinocchio;
using namespace std;

void addToBoneWeight(QuadPuppet::BoneWeight& boneWeight, int index, float weight)
{
    for (int i = 0; i < boneWeight.first.size(); i++)
    {
        if (boneWeight.second[i] < weight)
        {
            //Shift this slot and everything after it towards the end
            for (int j = boneWeight.first.size() - 1; j > i; j--)
            {
                boneWeight.first[j] = boneWeight.first[j - 1];
                boneWeight.second[j] = boneWeight.second[j - 1];
            }
            //Put new bone in this slot
            boneWeight.first[i] = index;
            boneWeight.second[i] = weight;
            break;
        }
    }
}

class AttachmentPrivate
{
public:
    AttachmentPrivate() {}
    virtual ~AttachmentPrivate() {}
    virtual Mesh deform(const Mesh& mesh, const vector<Transform<>>& transforms) const = 0;
    virtual Vector<double, -1> getWeights(int i) const = 0;
    virtual size_t getWeightsSize() const = 0;
    virtual AttachmentPrivate* clone() const = 0;
};

bool vectorInCone(const Vector3& v, const vector<Vector3>& ns)
{
    int i;
    Vector3 avg;
    for (i = 0; i < (int)ns.size(); ++i)
        avg += ns[i];

    return v.normalize() * avg.normalize() > 0.5;
}

class AttachmentPrivate1 : public AttachmentPrivate
{
public:
    AttachmentPrivate1() {}

    AttachmentPrivate1(const Mesh& mesh, const vector<int>& parents, const vector<Vector3>& match, const VisibilityTester* tester)
    {
        int i, j;
        int nv = mesh.vertices.size();
        //compute edges
        vector<vector<int>> edges(nv);

        for (i = 0; i < nv; ++i)
        {
            int cur, start;
            cur = start = mesh.vertices[i].edge;
            do
            {
                edges[i].push_back(mesh.edges[cur].vertex);
                cur = mesh.edges[mesh.edges[cur].prev].twin;
            } while (cur != start);
        }

        weights.resize(nv);

        int bones = parents.size() - 1;

        for (i = 0; i < nv; ++i) // initialize the weights vectors so they are big enough
            weights[i][bones] = 0.; // allocates additional storage for the "hips" bone.

        vector<vector<double>> boneDists(nv);
        vector<vector<bool>> boneVis(nv);

        vector<int> terminalBones;
        for (int i = 0; i <= bones; i++)
        {
            bool isTerminal = true;
            for (int j = 0; j <= bones; j++)
            {
                if (parents[j] == i)
                {
                    isTerminal = false;
                    break;
                }
            }
            if (isTerminal)
            {
                terminalBones.push_back(i);
            }
        }

        for (i = 0; i < nv; ++i)
        {
            boneDists[i].resize(bones, -1);
            boneVis[i].resize(bones);
            Vector3 cPos = mesh.vertices[i].pos;

            vector<Vector3> normals;
            for (j = 0; j < (int)edges[i].size(); ++j)
            {
                int nj = (j + 1) % edges[i].size();
                Vector3 v1 = mesh.vertices[edges[i][j]].pos - cPos;
                Vector3 v2 = mesh.vertices[edges[i][nj]].pos - cPos;
                normals.push_back((v1 % v2).normalize());
            }

            double minDist = 1e37;
            //Distance to bone joints as a point
            for (j = 1; j <= bones; ++j)
            {
                const Vector3& bonePos = match[j];
                boneDists[i][j - 1] = (cPos - bonePos).length();
                minDist = min(boneDists[i][j - 1], minDist);
            }
            //Distance to bones as segments
            for (j = 1; j <= bones; ++j)
            {
                int prevBoneIndex = parents[j];
                if (prevBoneIndex > 0)
                {
                    const Vector3 &v1 = match[j], &v2 = match[prevBoneIndex];
                    double dist = sqrt(distsqToSeg(cPos, v1, v2));
                    bool isTerminal = false;
                    for (int tBone = 0; tBone < terminalBones.size(); tBone++)
                    {
                        if (terminalBones[tBone] == j)
                        {
                            isTerminal = true;
                            break;
                        }
                    }

                    if (isTerminal)
                    {
                        //If vert is closer to the {current joint -> parent joint} bone than to the current joint
                        //reduce the {vert -> current joint} distance to match
                        boneDists[i][j - 1] = min(boneDists[i][j - 1], dist);
                    }
                    else
                    {
                        //If vert is closer to the {current joint -> parent joint} bone than to the parent joint itself
                        //reduce the {vert -> parent joint} distance to match
                        boneDists[i][prevBoneIndex - 1] = min(boneDists[i][prevBoneIndex - 1], dist);
                    }
                    minDist = min(dist, minDist);
                }
            }
            for (j = 1; j <= bones; ++j)
            {
                //the reason we don't just pick the closest bone is so that if two are
                //equally close, both are factored in.
                if (boneDists[i][j - 1] > minDist * 1.0001)
                    continue;

                const Vector3 &v1 = match[j], &v2 = match[parents[j]];
                Vector3 p = projToSeg(cPos, v1, v2);
                boneVis[i][j - 1] = tester->canSee(cPos, p) && vectorInCone(cPos - p, normals);
            }
        }

        //We have -Lw+Hw=HI, same as (H-L)w=HI, with (H-L)=DA (with D=diag(1./area))
        //so w = A^-1 (HI/D)

        vector<vector<pair<int, double>>> A(nv);
        vector<double> D(nv, 0.), H(nv, 0.);
        vector<int> closest(nv, -1);
        for (i = 0; i < nv; ++i)
        {
            //get areas
            for (j = 0; j < (int)edges[i].size(); ++j)
            {
                int nj = (j + 1) % edges[i].size();

                D[i] += ((mesh.vertices[edges[i][j]].pos - mesh.vertices[i].pos) % (mesh.vertices[edges[i][nj]].pos - mesh.vertices[i].pos)).length();
            }
            D[i] = 1. / (1e-10 + D[i]);

            //get bones
            double minDist = 1e37;
            for (j = 0; j < bones; ++j)
            {
                if (boneDists[i][j] < minDist)
                {
                    closest[i] = j;
                    minDist = boneDists[i][j];
                }
            }
            for (j = 0; j < bones; ++j)
                if (boneVis[i][j] && boneDists[i][j] <= minDist * 1.00001)
                    H[i] += 1. / SQR(1e-8 + boneDists[i][closest[i]]);

            //get laplacian
            double sum = 0.;
            for (j = 0; j < (int)edges[i].size(); ++j)
            {
                int nj = (j + 1) % edges[i].size();
                int pj = (j + edges[i].size() - 1) % edges[i].size();

                Vector3 v1 = mesh.vertices[i].pos - mesh.vertices[edges[i][pj]].pos;
                Vector3 v2 = mesh.vertices[edges[i][j]].pos - mesh.vertices[edges[i][pj]].pos;
                Vector3 v3 = mesh.vertices[i].pos - mesh.vertices[edges[i][nj]].pos;
                Vector3 v4 = mesh.vertices[edges[i][j]].pos - mesh.vertices[edges[i][nj]].pos;

                double cot1 = (v1 * v2) / (1e-6 + (v1 % v2).length());
                double cot2 = (v3 * v4) / (1e-6 + (v3 % v4).length());
                sum += (cot1 + cot2);

                if (edges[i][j] > i) //check for triangular here because sum should be computed regardless
                    continue;
                A[i].push_back(make_pair(edges[i][j], -cot1 - cot2));
            }

            A[i].push_back(make_pair(i, sum + H[i] / D[i]));

            sort(A[i].begin(), A[i].end());
        }

        nzweights.resize(nv);
        SPDMatrix Am(A);
        LLTMatrix* Ainv = Am.factor();
        if (Ainv == NULL)
            return;

        for (j = 0; j < bones; ++j)
        {
            vector<double> rhs(nv, 0.);
            for (i = 0; i < nv; ++i)
            {
                if (boneVis[i][j] && boneDists[i][j] <= boneDists[i][closest[i]] * 1.00001)
                    rhs[i] = H[i] / D[i];
            }

            Ainv->solve(rhs);
            for (i = 0; i < nv; ++i)
            {
                if (rhs[i] > 1.)
                    rhs[i] = 1.; //clip just in case
                if (rhs[i] > 1e-8)
                    nzweights[i].push_back(make_pair(j, rhs[i]));
            }
        }

        for (i = 0; i < nv; ++i)
        {
            double sum = 0.;
            for (j = 0; j < (int)nzweights[i].size(); ++j)
                sum += nzweights[i][j].second;

            for (j = 0; j < (int)nzweights[i].size(); ++j)
            {
                nzweights[i][j].second /= sum;
                // The bone index is incremented by 1
                // because the attachment omits the "hips" bone.
                weights[i][nzweights[i][j].first + 1] = nzweights[i][j].second;
            }
        }

        delete Ainv;
        return;
    }

    Mesh deform(const Mesh& mesh, const vector<Transform<>>& transforms) const
    {
        Mesh out = mesh;
        int i, nv = mesh.vertices.size();

        if (mesh.vertices.size() != weights.size())
            return out; //error

        for (i = 0; i < nv; ++i)
        {
            Vector3 newPos;
            int j;
            for (j = 0; j < (int)nzweights[i].size(); ++j)
            {
                newPos += ((transforms[nzweights[i][j].first] * out.vertices[i].pos) * nzweights[i][j].second);
            }
            out.vertices[i].pos = newPos;
        }

        out.computeVertexNormals();

        return out;
    }

    Vector<double, -1> getWeights(int i) const { return weights[i]; }
    size_t getWeightsSize() const { return weights.size(); }

    AttachmentPrivate* clone() const
    {
        AttachmentPrivate1* out = new AttachmentPrivate1();
        *out = *this;
        return out;
    }


    vector<vector<int>> getConnectedComponentsWithNoZeroWeight(const Mesh& mesh, set<int>& nonZeroVerts)
    {
        vector<vector<int>> connectedComponents = vector<vector<int>>();
        int currentcomponent = 0;
        int totalVertCount = mesh.vertices.size();
        
        while (!nonZeroVerts.empty())
        {
            // Get the first item and remove it
            int vertIndex = *(nonZeroVerts.begin());
            MeshVertex vert = mesh.vertices[vertIndex];
            nonZeroVerts.erase(nonZeroVerts.begin());

            // create a new component, add this vert
            connectedComponents.push_back(vector<int>(1, vertIndex));

            // walk the graph, adding every vert you can that has a weight > 0
            vector<int> todo(1, vertIndex);

            int inTodo = 0;
            while (inTodo < (int)todo.size())
            {
                // check verts in starting face
                // then flip on start edge, go next, check vert. repeat until
                int visitedCount = 0;

                int startEdge = mesh.vertices[todo[inTodo++]].edge;
                int curEdge = startEdge;
                do
                {
                    int vtx = mesh.edges[curEdge].vertex;
                    if (nonZeroVerts.count(vtx) != 0)
                    {
                        connectedComponents[currentcomponent].push_back(vtx);
                        nonZeroVerts.erase(vtx);
                        todo.push_back(vtx);
                    }
                    int prev = mesh.edges[curEdge].prev;
                    curEdge = mesh.edges[prev].twin; //walk around
                    visitedCount++;
                } while (curEdge != startEdge && curEdge != -1 && visitedCount < totalVertCount);

                // if we got to a -1 edge, we must loop the other way too
                if (curEdge == -1 && mesh.edges[startEdge].twin != -1)
                {
                    curEdge = mesh.edges[mesh.edges[mesh.edges[startEdge].twin].prev].prev;
                    do
                    {
                        int vtx = mesh.edges[curEdge].vertex;
                        if (nonZeroVerts.count(vtx) != 0)
                        {
                            connectedComponents[currentcomponent].push_back(vtx);
                            nonZeroVerts.erase(vtx);
                            todo.push_back(vtx);
                        }
                        if (mesh.edges[curEdge].twin != -1)
                        {
                            curEdge = mesh.edges[mesh.edges[mesh.edges[curEdge].twin].prev].prev;
                        }
                        else
                        {
                            curEdge = -1;
                        }
                        visitedCount++;
                    } while (curEdge != startEdge && curEdge != -1 && visitedCount < totalVertCount);
                }
                if (visitedCount >= totalVertCount)
                {
                    std::cout << "visitedCount exceeded totalCount in getConnectedComponentsWithNoZeroWeight. Half edge search failed. Probably non-manifold mesh was input." << endl; 
                    return connectedComponents;
                }
            }

            // new component time
            currentcomponent++;
        }
        return connectedComponents;
    }
    float calcComponentAverageDistance(vector<int> componentIndices, const Mesh& mesh, Vector3 headPos)
    {
        float sum = 0;
        for (int i = 0; i < componentIndices.size(); i++)
        {
            sum += (mesh.vertices[componentIndices[i]].pos - headPos).length();
        }
        return sum / (float)componentIndices.size();
    }

    //Handcrafted skin weighting for Boot scene of ISES.
    void DoBootAttachment(const Mesh& mesh, const vector<int>& parents, const vector<Vector3>& match, float fadeStart, float fadeEnd)
    {
        const int kShoulderIndex = 2;
        const int kHeadIndex = 3;
        int i, j;
        int nv = mesh.vertices.size();
        weights.resize(nv);

        int bones = parents.size() - 1;

        Vector3 shoulderPos = match[kShoulderIndex];
        Vector3 headPos = match[kHeadIndex];
        Vector3 shoulderLeftPos = match[12];
        Vector3 shoulderRightPos = match[15];

        set<int> HeadWeightIndices = set<int>();

        for (i = 0; i < nv; i++)
        {
            for (j = 0; j < bones; j++)
            {
                weights[i][j] = 0;
            }

            Vector3 vPos = mesh.vertices[i].pos;

            //Base weight: Parameterize along shoulder->head segment, start fading out near shoulder joint
            Vector3 projPos = projToSeg(vPos, shoulderPos, headPos);

            double p = (projPos - shoulderPos).length() / (headPos - shoulderPos).length();
            double headWeight = 0;

            if (p > fadeStart)
            {
                headWeight = 1;
            }
            else if (p < fadeEnd)
            {
                headWeight = 0;
            }
            else
            {
                headWeight = (p - fadeEnd) / (fadeStart - fadeEnd);
                headWeight = headWeight * headWeight;
            }

            //Modifier: If closer to a shoulder segment than the neck segment, rapidly fade out
            double distLeftShoulderSeg = sqrt(distsqToSeg(vPos, shoulderPos, shoulderLeftPos));
            double distRightShoulderSeg = sqrt(distsqToSeg(vPos, shoulderPos, shoulderRightPos));
            double distNeckSeg = sqrt(distsqToSeg(vPos, shoulderPos, headPos));

            double distShoulderSegMin = min(distLeftShoulderSeg, distRightShoulderSeg);
            if (distShoulderSegMin < distNeckSeg)
            {
                //As far away from neck as shoulder: no modifier
                //>20% further away from neck than shoulder: 0 weight
                double ratio = distNeckSeg / distShoulderSegMin;
                double multiplier = 1 - 5 * (ratio - 1);
                if (multiplier <= 0)
                {
                    headWeight = 0;
                }
                else
                {
                    headWeight *= multiplier;
                }
            }

            if (headWeight > 0)
            {
                HeadWeightIndices.insert(i);
            }
            weights[i][kHeadIndex] = headWeight;
            weights[i][kShoulderIndex] = 1 - headWeight;
        }
        
        auto connectedComponents = getConnectedComponentsWithNoZeroWeight(mesh, HeadWeightIndices);
        
        vector<vector<int>>::iterator bestIter = connectedComponents.end();
        float best = numeric_limits<float>::infinity();
        for (auto iter = connectedComponents.begin(); iter != connectedComponents.end(); ++iter)
        {
            if (iter->size() > MIN_COMPONENT_SIZE_FOR_HEAD)
            {
                float avgDist = calcComponentAverageDistance(*iter, mesh, headPos);
                if (avgDist < best)
                {
                    bestIter = iter;
                    best = avgDist;
                }
            }
        }

        // For not closest connected components, fully weight to the shoulders
        for (auto iter = connectedComponents.begin(); iter != connectedComponents.end(); ++iter)
        {
            if (iter != bestIter)
            {
                for (int i = 0; i < iter->size(); i++)
                {
                    int index = (*iter)[i];
                    weights[index][kHeadIndex] = 0;
                    weights[index][kShoulderIndex] = 1;
                }
            }
        }
    }

private:
    vector<Vector<double, -1>> weights;
    vector<vector<pair<int, double>>> nzweights; //sparse representation
};

Attachment::~Attachment()
{
    if (a)
        delete a;
}

Attachment::Attachment(const Attachment& att)
{
    a = att.a->clone();
}

Vector<double, -1> Attachment::getWeights(int i) const
{
    return a->getWeights(i);
}
size_t Attachment::getWeightsSize() const
{
    return a->getWeightsSize();
}

//Converts from the internal format (bones)x(weight) to a 4 x (bone, weight) format
std::vector<QuadPuppet::BoneWeight> Attachment::getBoneWeights()
{
    std::vector<QuadPuppet::BoneWeight> converted;
    for (int v = 0; v < getWeightsSize(); v++)
    {
        Vector<double, -1> weights = getWeights(v);
        QuadPuppet::BoneWeight boneWeight;
        for (int i = 0; i < 4; i++)
        {
            boneWeight.first[i] = -1;
            boneWeight.second[i] = 0;
        }
        for (int w = 0; w < weights.size(); w++)
        {
            addToBoneWeight(boneWeight, w, weights[w]);
        }
        converted.push_back(boneWeight);
    }
    return converted;
}

Mesh Attachment::deform(const Mesh& mesh, const vector<Transform<>>& transforms) const
{
    return a->deform(mesh, transforms);
}

Attachment::Attachment(const Mesh& mesh, const vector<int>& parents, const vector<Vector3>& match, const VisibilityTester* tester, const QuadPuppet::RiggingParameters::RiggingSource rigType, const QuadPuppet::RiggingParameters& params)
{
    auto source = rigType;
    if (source == QuadPuppet::RiggingParameters::RiggingSource::PINOCCHIO)
    {
        a = new AttachmentPrivate1(mesh, parents, match, tester);
    }
    else if (source == QuadPuppet::RiggingParameters::RiggingSource::PINOCCHIO_HEAD_SKIN_ONLY)
    {
        a = new AttachmentPrivate1();
        ((AttachmentPrivate1*)a)->DoBootAttachment(mesh, parents, match, params.headSkinningStartFadeDistance, params.headSkinningEndFadeDistance);
    }
    else
    {
        std::cout << "Invalid rig source for pinocchio attachment: " << source << std::endl;
        throw std::exception("Cannot create pinocchio attachment from specified rigging source");
    }
}
