#pragma once

#include "URI.h"
#include <models/SerializedEmbedding.h>
#include <models/mesh_t.h>
#include "vector.h"

namespace arcturus {
using namespace arcturus::holoedit;
namespace hps {
namespace interchange {

// Return a vec3_t from a Vector3 (Pinocchio). Results in a loss of precision (double to float).
vec3_t to_vec3(Vector3 v);

// Return a Vector3 (Pinocchio) from a vec3_t.
Vector3 to_Vector3(vec3_t v);

// Return a vec4_t from a Vector4 (Pinocchio). Results in a loss of precision (double to float).
vec4_t to_vec4(Vector4 v);

template<typename T>
T* adapt(mesh_t* mesh);

template<typename T>
T* read(URI uri);

}
}
}