#pragma once

#include <Eigen/Core>
#include <vector>
#include <map>

#ifdef QP_DLL_EXPORT
/*Enabled as "export" while compiling the dll project*/
#define DLLEXPORT __declspec(dllexport)  
#else
/*Enabled as "import" in the Client side for using already created dll file*/
#define DLLEXPORT __declspec(dllimport)  
#endif

namespace QuadPuppet
{
    union vec2_t {
        struct
        {
            float x, y;
        };
        struct
        {
            float u, v;
        };
        float data[2];
    };

    union vec3_t {
        struct
        {
            float x, y, z;
        };
        struct
        {
            float u, v, _;
        };
        struct
        {
            float r, g, b;
        };
        float data[3];
    };

    union vec4_t {
        struct
        {
            float x, y, z, w;
        };
        struct
        {
            float u, v, _, __;
        };
        struct
        {
            float r, g, b, a;
        };
        float data[4];
    };

    typedef std::pair<std::array<int, 4>, std::array<float, 4>> BoneWeight;

    struct RiggingParameters
    {
        bool useSmoothPF;
        bool skinAndSave;
        bool postSmooth;
        float headSkinningStartFadeDistance;
        float headSkinningEndFadeDistance;

        enum RiggingSource {
            INVALID, //Non-source / error
            RIGGED, //Read from a previously-generated frameN.dat rigging binary data file in ./Rigged
            FILE_JSON, //Read from a frameN.json file in ./Skeleton (joints only)
            FILE_FBX, //Read from a frameN.fbx or framesN-M.fbx file in ./Skeleton
            PINOCCHIO, //Generate using pinocchio
            OPENPOSE, //Generate using openpose (joints only)
            PINOCCHIO_HEAD_SKIN_ONLY, //Generate using pinnochio with the weighting hack just for Boot's turn-and-look section (weights only)
            LAST //Number of values in the enum, not "previous data"
        };
    };

    struct Embedding
    {
    public:
        // A root joint has no parent.
        static const int kRootJointIndex = -1;

        std::vector<std::string> joints; //List of joint names. The index of the joint name in the list is the "joint index" for that joint
        std::vector<int> parents; //For joints[i], the parent of that joint is joints[parents[i]]. If parents[i] is <0, joint[i] is a root joint / not parented.
        std::map<std::string, vec3_t> embedding; //Bone name -> Bone position [model / global space]
        std::map<std::string, vec3_t> localEmbedding; //Bone name - > Bone position [Relative to parent bone]
        std::map<std::string, bool> embeddingKeyframe; //Joint name -> embedding is a keyframe (not interpolatable from nearby frames)
        std::map<std::string, vec4_t> localRotations; //Bone name -> Bone rotation [Quaternion. Relative to parent bone]
        std::map<std::string, bool> rotationsKeyframe; //Joint name -> local rotation is a keyframe

        inline void flipXAxis()
        {
            for (int i = 0; i < joints.size(); ++i)
            {
                std::string jointName = joints[i];

                // Flip the X-Axis in the embeddings
                embedding[jointName].x = -embedding[jointName].x;
                localEmbedding[jointName].x = -localEmbedding[jointName].x;

                // Flip the X-Axis in the rotation
                localRotations[jointName].x = -localRotations[jointName].x;
                localRotations[jointName].w = -localRotations[jointName].w;
            }
        }
    };

    DLLEXPORT double SolveQP(const Eigen::MatrixXd& gm, const Eigen::VectorXd& gv,
        const Eigen::MatrixXd& cem, const Eigen::VectorXd& cev,
        const Eigen::MatrixXd& cim, const Eigen::VectorXd& civ,
        Eigen::VectorXd& xv);

    DLLEXPORT std::vector<BoneWeight> GetAttachmentWeights(std::string meshPath, Embedding* embedding, RiggingParameters::RiggingSource rigType, RiggingParameters& params);

    DLLEXPORT bool GetEmbeddingForFrameFromPinocchio(std::string meshPath, Embedding* embedding);
}