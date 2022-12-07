#include "QuadPuppet/QuadPuppet.h"
#include "quadprog/QuadProg.h"
#include "mesh.h"
#include "pinocchioApi.h"

namespace QuadPuppet
{
    double SolveQP(const Eigen::MatrixXd& gm, const Eigen::VectorXd& gv,
        const Eigen::MatrixXd& cem, const Eigen::VectorXd& cev,
        const Eigen::MatrixXd& cim, const Eigen::VectorXd& civ,
        Eigen::VectorXd& xv)
    {
        return ::SolveQP(gm, gv, cem, cev, cim, civ, xv);
    }

    Vector3 to_Vector3(vec3_t v)
    {
        return { v.data[0], v.data[1], v.data[2] };
    }

    vec3_t to_vec3(Vector3 v)
    {
        return { (float)v[0], (float)v[1], (float)v[2] };
    }

    vec4_t to_vec4(Vector4 v)
    {
        return { (float)v[0], (float)v[1], (float)v[2], (float)v[3] };
    }

    void AdaptMesh(Pinocchio::Mesh* mesh)
    {
        if (mesh != NULL)
        {
            // Make Believe numbers.
            float x = 1.0f;
            float y = 0.0f;
            float z = 0.0f;
            float deg = 0.0f;

            Quaternion<> meshTransform;

            // Magic rotation to make it have Y positive value
            meshTransform = Quaternion<>(Vector3(x, y, z), deg * M_PI / 180.) * meshTransform;

            // for each vert transform by the mesh transform,
            // this is if there exists a manual rotation.
            for (int i = 0; i < (int)mesh->vertices.size(); ++i)
            {
                mesh->vertices[i].pos = meshTransform * mesh->vertices[i].pos;
            }

            mesh->normalizeBoundingBox();
            mesh->computeVertexNormals();
        }
    }

    std::vector<Vector3> embeddingInPinochioCoords(Embedding& info, Pinocchio::Mesh& mesh)
    {
        std::vector<Vector3> pinEmbed;
        for (size_t i = 0; i < info.joints.size(); i++)
        {
            Vector3 pinPos = to_Vector3(info.embedding[info.joints[i]]);
            pinPos = pinPos * mesh.scale + mesh.toAdd; //Apply scaling
            pinEmbed.push_back(pinPos);
        }
        return pinEmbed;
    }

    Attachment* CreateAttachment(Embedding& embed, Pinocchio::Mesh& mesh, const QuadPuppet::RiggingParameters::RiggingSource rigType, const QuadPuppet::RiggingParameters& params)
    {
        TreeType* distanceField = constructDistanceField(mesh);
        VisTester<TreeType>* tester = new VisTester<TreeType>(distanceField);
        Attachment* createdAttachment = new Attachment(mesh, embed.parents, embeddingInPinochioCoords(embed, mesh), tester, rigType, params);
        //cleanup
        delete distanceField;
        delete tester;

        return createdAttachment;
    }

    std::vector<BoneWeight> GetAttachmentWeights(std::string meshPath, Embedding* embedding, QuadPuppet::RiggingParameters::RiggingSource rigType, QuadPuppet::RiggingParameters& params)
    {
        std::vector<BoneWeight> welded_weights;

        // Load Mesh
        Pinocchio::Mesh* mesh = new Pinocchio::Mesh(meshPath);
        if (mesh == nullptr)
        {
            return welded_weights;
        }
        AdaptMesh(mesh);

        Attachment* attachment = CreateAttachment(*embedding, *mesh, rigType, params);

        welded_weights = attachment->getBoneWeights();
        return welded_weights;
    }

    void fillEmbeddingFromPinocchioEmbedding(std::vector<Vector3> pinochioEmbedding, Pinocchio::Mesh* mesh, Skeleton& skeleton, Embedding* info)
    {
        for (size_t i = 0; i < skeleton.names().size(); i++)
        {
            info->joints.push_back(skeleton.names()[i]);
        }
        for (size_t i = 0; i < skeleton.fPrev().size(); i++)
        {
            info->parents.push_back(skeleton.fPrev()[i]);
        }

        info->embedding.clear();
        for (size_t i = 0; i < pinochioEmbedding.size(); ++i)
        {
            Vector3 pos = (pinochioEmbedding[i] - mesh->toAdd) / mesh->scale; //Undo scaling
            pos[0] = -pos[0]; //Pinocchio X coord is mirrored
            info->embedding[info->joints[i]] = to_vec3(pos);
        }

        info->localEmbedding.clear();
        for (size_t i = 0; i < info->joints.size(); ++i)
        {
            Vector3 pos = to_Vector3(info->embedding[info->joints[i]]);
            Vector3 parentPos;
            if (info->parents[i] >= 0)
            {
                parentPos = to_Vector3(info->embedding[info->joints[info->parents[i]]]);
            }
            else
            {
                parentPos = Vector3(0.f, 0.f, 0.f); //No parent, local coord == global coord
            }
            info->localEmbedding[info->joints[i]] = to_vec3(pos - parentPos);
        }

        info->localRotations.clear();
        for (size_t i = 0; i < info->joints.size(); i++)
        {
            //Pinocchio does not output rotations -- fill with identity rots
            Vector4 quaternion;
            quaternion[0] = quaternion[1] = quaternion[2] = 0;
            quaternion[3] = 1; //w is last
            info->localRotations[info->joints[i]] = to_vec4(quaternion);
        }

        info->embeddingKeyframe.clear();
        info->rotationsKeyframe.clear();
        //All frames are keyframes
        for (size_t i = 0; i < info->joints.size(); i++)
        {
            info->embeddingKeyframe[info->joints[i]] = true;
            info->rotationsKeyframe[info->joints[i]] = true;
        }
        info->flipXAxis(); //Convert from LH to RH
    }

    bool GetEmbeddingForFrameFromPinocchio(std::string meshPath, Embedding* embedding)
    {
        Pinocchio::Mesh* mesh = new Pinocchio::Mesh(meshPath);
        if (!mesh)
        {
            std::cout << "Pinocchio embed not possible: Mesh " << meshPath << " not found" << std::endl;
            return false;
        }

        AdaptMesh(mesh);

        Embedding* resultEmbed = NULL;
        Skeleton skel = HumanSkeleton::HumanSkeleton();

        PinocchioOutput o = autorig(skel, *mesh);
        fillEmbeddingFromPinocchioEmbedding(o.embedding, mesh, skel, embedding);
        if (skel.prevSkeleton)
        {
            delete skel.prevSkeleton;
        }
        delete mesh;
        return true;
    }
}