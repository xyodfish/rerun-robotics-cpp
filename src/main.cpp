#include <rerun.hpp>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cctype>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

struct MeshData {
  std::vector<Eigen::Vector3f> vertices;
  std::vector<Eigen::Vector3f> normals;
  std::vector<Eigen::Vector3i> triangles;
};

struct VisualInstance {
  std::string entity_path;
  pinocchio::FrameIndex parent_frame_id{};
  std::string parent_frame_name;
  pinocchio::SE3 visual_placement;
  Eigen::Vector3d scale = Eigen::Vector3d::Ones();
  MeshData mesh;
};

static Eigen::Vector3f toVec3f(const aiVector3D &v) {
  return Eigen::Vector3f(v.x, v.y, v.z);
}

static std::string sanitizeName(const std::string &s) {
  std::string out = s;
  for (auto &c : out) {
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' ||
          c == '/')) {
      c = '_';
    }
  }
  return out;
}

static std::string resolveMeshPath(const std::string &mesh_uri,
                                   const std::string &urdf_path,
                                   const std::string &mesh_root_dir) {
  fs::path p(mesh_uri);

  if (p.is_absolute()) {
    return p.string();
  }

  const std::string prefix = "package://";
  if (mesh_uri.rfind(prefix, 0) == 0) {
    auto rest = mesh_uri.substr(prefix.size());
    auto slash_pos = rest.find('/');
    if (slash_pos != std::string::npos) {
      auto rel = rest.substr(slash_pos + 1); // meshes/...
      return (fs::path(mesh_root_dir) /
              fs::path(rel).lexically_relative("meshes"))
          .string();
    }
  }

  return (fs::path(urdf_path).parent_path() / p).string();
}

static std::vector<rerun::components::Vector3D>
toRerunNormals(const std::vector<Eigen::Vector3f> &normals) {
  std::vector<rerun::components::Vector3D> out;
  out.reserve(normals.size());
  for (const auto &n : normals) {
    out.emplace_back(n.x(), n.y(), n.z());
  }
  return out;
}

static MeshData loadMeshWithAssimp(const std::string &mesh_file,
                                   const Eigen::Vector3d &scale) {
  Assimp::Importer importer;
  const aiScene *scene = importer.ReadFile(
      mesh_file, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
                     aiProcess_ImproveCacheLocality | aiProcess_SortByPType |
                     aiProcess_GenSmoothNormals);

  if (!scene || !scene->HasMeshes()) {
    throw std::runtime_error("Failed to load mesh: " + mesh_file);
  }

  MeshData out;
  size_t vertex_offset = 0;

  for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
    const aiMesh *mesh = scene->mMeshes[m];
    if (!mesh || !mesh->HasPositions()) {
      continue;
    }

    for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
      Eigen::Vector3f p = toVec3f(mesh->mVertices[v]);
      p.x() *= static_cast<float>(scale.x());
      p.y() *= static_cast<float>(scale.y());
      p.z() *= static_cast<float>(scale.z());
      out.vertices.push_back(p);
      Eigen::Vector3f n(0.0f, 0.0f, 1.0f);
      if (mesh->HasNormals()) {
        n = toVec3f(mesh->mNormals[v]);
        if (n.norm() > 1e-8f) {
          n.normalize();
        }
      }
      out.normals.push_back(n);
    }

    for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
      const aiFace &face = mesh->mFaces[f];
      if (face.mNumIndices != 3) {
        continue;
      }
      out.triangles.emplace_back(
          static_cast<int>(vertex_offset + face.mIndices[0]),
          static_cast<int>(vertex_offset + face.mIndices[1]),
          static_cast<int>(vertex_offset + face.mIndices[2]));
    }

    vertex_offset += mesh->mNumVertices;
  }

  if (out.vertices.empty() || out.triangles.empty()) {
    throw std::runtime_error("Mesh has no valid triangles: " + mesh_file);
  }

  return out;
}

static std::vector<VisualInstance> loadVisualsFromUrdf(
    const std::string &urdf_path, const std::string &package_root_dir,
    const std::string &mesh_root_dir, const pinocchio::Model &model) {
  pinocchio::GeometryModel geom_model;
  pinocchio::urdf::buildGeom(model, urdf_path, pinocchio::GeometryType::VISUAL,
                             geom_model, package_root_dir);

  std::vector<VisualInstance> visuals;
  visuals.reserve(geom_model.geometryObjects.size());

  for (const auto &go : geom_model.geometryObjects) {
    if (go.meshPath.empty()) {
      continue;
    }

    VisualInstance vis;
    vis.parent_frame_id = go.parentFrame;
    vis.parent_frame_name = model.frames[go.parentFrame].name;
    vis.visual_placement = go.placement;
    vis.scale = go.meshScale;

    const std::string mesh_path =
        resolveMeshPath(go.meshPath, urdf_path, mesh_root_dir);
    vis.mesh = loadMeshWithAssimp(mesh_path, vis.scale);

    vis.entity_path = "robot/visuals/" + sanitizeName(go.name);
    visuals.push_back(std::move(vis));
  }

  return visuals;
}

static std::vector<rerun::Position3D>
toRerunPositions(const std::vector<Eigen::Vector3f> &pts) {
  std::vector<rerun::Position3D> out;
  out.reserve(pts.size());
  for (const auto &p : pts) {
    out.emplace_back(p.x(), p.y(), p.z());
  }
  return out;
}

static std::vector<rerun::components::TriangleIndices>
toRerunTriangles(const std::vector<Eigen::Vector3i> &tris) {
  std::vector<rerun::components::TriangleIndices> out;
  out.reserve(tris.size());
  for (const auto &t : tris) {
    out.emplace_back(static_cast<uint32_t>(t.x()), static_cast<uint32_t>(t.y()),
                     static_cast<uint32_t>(t.z()));
  }
  return out;
}

int main() {
  const std::string urdf_path =
      "/data/config/galbot_description/galbot_one_golf_description/"
      "galbot_one_golf.urdf";
  const std::string package_root_dir = "/data/config/galbot_description";
  const std::string mesh_root_dir =
      "/data/config/galbot_description/galbot_one_golf_description/meshes";

  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_path, model);
  pinocchio::Data data(model);

  std::cout << "[INFO] model nq=" << model.nq << ", nv=" << model.nv
            << std::endl;

  auto visuals =
      loadVisualsFromUrdf(urdf_path, package_root_dir, mesh_root_dir, model);
  std::cout << "[INFO] loaded visuals: " << visuals.size() << std::endl;

  auto rec = rerun::RecordingStream("galbot_mesh_static");
  rec.spawn().exit_on_failure();

  rec.log("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

  //   {
  //     std::vector<rerun::Position3D> pts{rerun::Position3D(0.0f, 0.0f, 0.0f),
  //                                        rerun::Position3D(0.5f, 0.0f, 0.0f),
  //                                        rerun::Position3D(0.0f, 0.5f, 0.0f),
  //                                        rerun::Position3D(0.0f, 0.0f,
  //                                        0.5f)};
  //     rec.log("world/debug_points",
  //             rerun::Points3D(pts).with_radii(
  //                 std::vector<float>{0.06f, 0.04f, 0.04f, 0.04f}));
  //   }

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  if (model.nq >= 7 && model.joints[1].nq() == 7) {
    q[6] = 1.0;
  }

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  std::cout << "[INFO] start logging static visuals..." << std::endl;

  for (size_t i = 0; i < visuals.size(); ++i) {
    const auto &vis = visuals[i];

    auto verts = toRerunPositions(vis.mesh.vertices);
    auto normals = toRerunNormals(vis.mesh.normals);
    auto tris = toRerunTriangles(vis.mesh.triangles);

    std::cout << "[INFO] mesh entity=" << vis.entity_path
              << " parent_frame=" << vis.parent_frame_name
              << " verts=" << verts.size() << " tris=" << tris.size()
              << std::endl;

    rec.log(
        vis.entity_path,
        rerun::Mesh3D(verts)
            .with_triangle_indices(tris)
            .with_vertex_normals(normals)
            .with_albedo_factor(rerun::components::AlbedoFactor(0xB4B4B4FF)));

    const pinocchio::FrameIndex fid = vis.parent_frame_id;
    if (fid >= data.oMf.size()) {
      std::cerr << "[WARN] invalid parent frame id for: " << vis.entity_path
                << std::endl;
      continue;
    }

    const pinocchio::SE3 &oMf = data.oMf[fid];
    const pinocchio::SE3 oMv = oMf * vis.visual_placement;

    const Eigen::Vector3d txyz = oMv.translation();
    const Eigen::Quaterniond qrot(oMv.rotation());

    std::cout << "[INFO] transform entity=" << vis.entity_path << " t=["
              << txyz.transpose() << "]"
              << " q=[" << qrot.x() << ", " << qrot.y() << ", " << qrot.z()
              << ", " << qrot.w() << "]" << std::endl;

    rec.log(vis.entity_path,
            rerun::Transform3D::from_translation_rotation(
                {static_cast<float>(txyz.x()), static_cast<float>(txyz.y()),
                 static_cast<float>(txyz.z())},
                rerun::Quaternion::from_xyzw(static_cast<float>(qrot.x()),
                                             static_cast<float>(qrot.y()),
                                             static_cast<float>(qrot.z()),
                                             static_cast<float>(qrot.w()))));
  }

  std::cout << "[INFO] static robot logged." << std::endl;

  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}