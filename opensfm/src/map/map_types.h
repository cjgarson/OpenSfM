#pragma once
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/StdVector>

namespace map {
template <typename K, typename V>
using AlignedUnorderedMap =
    std::unordered_map<K, V,
                       std::hash<K>, std::equal_to<K>,
                       Eigen::aligned_allocator<std::pair<const K, V>>>;

// Forward declarations
struct RigCamera;
struct RigInstance;
class Shot;
class Landmark;
namespace geometry {
class Camera;
class Similarity;
}

// Concrete aliases
using CameraMap       = AlignedUnorderedMap<std::string, geometry::Camera>;
using BiasMap         = AlignedUnorderedMap<std::string, geometry::Similarity>;
using ShotMap         = AlignedUnorderedMap<std::string, Shot>;
using LandmarkMap     = AlignedUnorderedMap<std::string, Landmark>;
using RigCameraMap    = AlignedUnorderedMap<std::string, RigCamera>;
using RigInstanceMap  = AlignedUnorderedMap<std::string, RigInstance>;

}  // namespace map
