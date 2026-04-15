#include <pinocchio/algorithm/joint-configuration.hpp>

#include <rerun.hpp>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
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
  std::shared_ptr<const MeshData> mesh;
};

struct JointMotion {
  std::string name;
  int q_idx = -1;
  double q0 = 0.0;
  double amp = 0.2;
  double freq = 0.2;
  double phase = 0.0;
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
      auto rel = rest.substr(slash_pos + 1);
      return (fs::path(mesh_root_dir) /
              fs::path(rel).lexically_relative("meshes"))
          .string();
    }
  }

  return (fs::path(urdf_path).parent_path() / p).string();
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

static std::string meshCacheKey(const std::string &mesh_file,
                                const Eigen::Vector3d &scale) {
  const auto sx = static_cast<long long>(std::llround(scale.x() * 1e9));
  const auto sy = static_cast<long long>(std::llround(scale.y() * 1e9));
  const auto sz = static_cast<long long>(std::llround(scale.z() * 1e9));
  return mesh_file + "|" + std::to_string(sx) + "," + std::to_string(sy) +
         "," + std::to_string(sz);
}

static std::vector<VisualInstance> loadVisualsFromUrdf(
    const std::string &urdf_path, const std::string &package_root_dir,
    const std::string &mesh_root_dir, const pinocchio::Model &model) {
  const auto t0 = std::chrono::steady_clock::now();

  (void)package_root_dir;

  const auto urdf_tree = urdf::parseURDFFile(urdf_path);
  if (!urdf_tree) {
    throw std::runtime_error("Failed to parse URDF: " + urdf_path);
  }

  struct PreparedVisual {
    std::string entity_path;
    pinocchio::FrameIndex parent_frame_id{};
    std::string parent_frame_name;
    pinocchio::SE3 visual_placement;
    Eigen::Vector3d scale = Eigen::Vector3d::Ones();
    size_t mesh_idx = 0;
  };

  struct MeshRequest {
    std::string mesh_path;
    Eigen::Vector3d scale = Eigen::Vector3d::Ones();
  };

  std::unordered_map<std::string, size_t> mesh_key_to_idx;
  std::vector<MeshRequest> unique_meshes;
  unique_meshes.reserve(64);

  std::vector<PreparedVisual> prepared_visuals;
  prepared_visuals.reserve(64);

  std::vector<VisualInstance> visuals;
  visuals.reserve(64);

  for (size_t fid = 0; fid < model.frames.size(); ++fid) {
    const auto &frame = model.frames[fid];
    if (frame.type != pinocchio::BODY) {
      continue;
    }

    const auto link = urdf_tree->getLink(frame.name);
    if (!link) {
      continue;
    }

    std::vector<urdf::VisualSharedPtr> link_visuals;
    if (!link->visual_array.empty()) {
      link_visuals = link->visual_array;
    } else if (link->visual) {
      link_visuals.push_back(link->visual);
    }

    for (size_t vis_idx = 0; vis_idx < link_visuals.size(); ++vis_idx) {
      const auto &visual = link_visuals[vis_idx];
      if (!visual || !visual->geometry ||
          visual->geometry->type != urdf::Geometry::MESH) {
        continue;
      }

      const auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(visual->geometry);
      if (!mesh || mesh->filename.empty()) {
        continue;
      }

      Eigen::Vector3d mesh_scale = Eigen::Vector3d::Ones();
      mesh_scale.x() = mesh->scale.x;
      mesh_scale.y() = mesh->scale.y;
      mesh_scale.z() = mesh->scale.z;
      if ((mesh_scale.array() == 0.0).all()) {
        mesh_scale = Eigen::Vector3d::Ones();
      }

      const std::string mesh_path =
          resolveMeshPath(mesh->filename, urdf_path, mesh_root_dir);
      const std::string key = meshCacheKey(mesh_path, mesh_scale);

      size_t mesh_idx = 0;
      const auto it = mesh_key_to_idx.find(key);
      if (it == mesh_key_to_idx.end()) {
        mesh_idx = unique_meshes.size();
        unique_meshes.push_back(MeshRequest{mesh_path, mesh_scale});
        mesh_key_to_idx.emplace(key, mesh_idx);
      } else {
        mesh_idx = it->second;
      }

      Eigen::Vector3d txyz(visual->origin.position.x, visual->origin.position.y,
                           visual->origin.position.z);
      Eigen::Quaterniond qrot(visual->origin.rotation.w, visual->origin.rotation.x,
                              visual->origin.rotation.y, visual->origin.rotation.z);
      if (qrot.norm() < 1e-8) {
        qrot = Eigen::Quaterniond::Identity();
      } else {
        qrot.normalize();
      }

      PreparedVisual prepared;
      prepared.entity_path =
          "robot/visuals/" + sanitizeName(frame.name + "_" + std::to_string(vis_idx));
      prepared.parent_frame_id = static_cast<pinocchio::FrameIndex>(fid);
      prepared.parent_frame_name = frame.name;
      prepared.visual_placement = pinocchio::SE3(qrot.toRotationMatrix(), txyz);
      prepared.scale = mesh_scale;
      prepared.mesh_idx = mesh_idx;
      prepared_visuals.push_back(std::move(prepared));
    }
  }

  const auto t1 = std::chrono::steady_clock::now();

  std::vector<std::shared_ptr<const MeshData>> loaded_meshes;
  loaded_meshes.resize(unique_meshes.size());

  const size_t max_workers =
      std::max<size_t>(1, static_cast<size_t>(std::thread::hardware_concurrency()));
  std::deque<std::pair<size_t, std::future<MeshData>>> in_flight;

  auto drainOne = [&]() {
    auto &front = in_flight.front();
    auto mesh = front.second.get();
    loaded_meshes[front.first] = std::make_shared<MeshData>(std::move(mesh));
    in_flight.pop_front();
  };

  for (size_t i = 0; i < unique_meshes.size(); ++i) {
    const auto mesh_path = unique_meshes[i].mesh_path;
    const auto scale = unique_meshes[i].scale;

    in_flight.emplace_back(i, std::async(std::launch::async, [mesh_path, scale]() {
                           return loadMeshWithAssimp(mesh_path, scale);
                         }));

    if (in_flight.size() >= max_workers) {
      drainOne();
    }
  }

  while (!in_flight.empty()) {
    drainOne();
  }

  for (const auto &prepared : prepared_visuals) {
    VisualInstance vis;
    vis.entity_path = prepared.entity_path;
    vis.parent_frame_id = prepared.parent_frame_id;
    vis.parent_frame_name = prepared.parent_frame_name;
    vis.visual_placement = prepared.visual_placement;
    vis.scale = prepared.scale;
    vis.mesh = loaded_meshes[prepared.mesh_idx];
    visuals.push_back(std::move(vis));
  }

  const auto t2 = std::chrono::steady_clock::now();
  const auto prep_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  const auto load_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  std::cout << "[INFO] visual prep: " << prep_ms << " ms, mesh load: " << load_ms
            << " ms, visuals: " << visuals.size()
            << ", unique meshes: " << unique_meshes.size() << std::endl;

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

static std::vector<rerun::components::Vector3D>
toRerunNormals(const std::vector<Eigen::Vector3f> &normals) {
  std::vector<rerun::components::Vector3D> out;
  out.reserve(normals.size());
  for (const auto &n : normals) {
    out.emplace_back(n.x(), n.y(), n.z());
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

static std::vector<rerun::Position3D> onePoint(const Eigen::Vector3d &p) {
  std::vector<rerun::Position3D> pts;
  pts.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()),
                   static_cast<float>(p.z()));
  return pts;
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

  auto rec = rerun::RecordingStream("galbot_mesh_anim");
  rec.spawn().exit_on_failure();

  // Keep all spatial data on the same timeline so temporal views always have geometry.
  rec.set_time_sequence("frame", 0);
  rec.log("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

  // mesh 只发一次（frame=0），后续帧只更新 transform
  for (const auto &vis : visuals) {
    if (!vis.mesh) {
      continue;
    }
    auto verts = toRerunPositions(vis.mesh->vertices);
    auto normals = toRerunNormals(vis.mesh->normals);
    auto tris = toRerunTriangles(vis.mesh->triangles);

    rec.log(
        vis.entity_path,
        rerun::Mesh3D(verts)
            .with_triangle_indices(tris)
            .with_vertex_normals(normals)
            .with_albedo_factor(rerun::components::AlbedoFactor(0xB4B4B4FF)));
  }

  Eigen::VectorXd q = pinocchio::neutral(model);

  const std::vector<std::string> joint_names = {
      "left_arm_joint1", "left_arm_joint2", "left_arm_joint3",
      "left_arm_joint4", "left_arm_joint5", "left_arm_joint6",
      "left_arm_joint7"};

  std::vector<JointMotion> motions;
  motions.reserve(joint_names.size());

  for (size_t i = 0; i < joint_names.size(); ++i) {
    const auto jid = model.getJointId(joint_names[i]);
    if (jid == 0) {
      std::cerr << "[WARN] joint not found: " << joint_names[i] << std::endl;
      continue;
    }

    const auto &jmodel = model.joints[jid];
    if (jmodel.nq() != 1) {
      std::cerr << "[WARN] joint nq != 1, skip: " << joint_names[i]
                << std::endl;
      continue;
    }

    JointMotion jm;
    jm.name = joint_names[i];
    jm.q_idx = jmodel.idx_q();
    jm.q0 = q[jm.q_idx];
    jm.amp = 0.18 + 0.02 * static_cast<double>(i);
    jm.freq = 0.18 + 0.015 * static_cast<double>(i);
    jm.phase = 0.35 * static_cast<double>(i);

    motions.push_back(jm);

    std::cout << "[INFO] animate joint " << jm.name << " q_idx=" << jm.q_idx
              << std::endl;
  }

  for (int frame = 0;; ++frame) {
    const double t = 0.02 * static_cast<double>(frame);
    rec.set_time_sequence("frame", frame);

    for (const auto &jm : motions) {
      q[jm.q_idx] =
          jm.q0 + jm.amp * std::sin(2.0 * M_PI * jm.freq * t + jm.phase);
    }

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    for (const auto &vis : visuals) {
      const pinocchio::FrameIndex fid = vis.parent_frame_id;
      if (fid >= data.oMf.size()) {
        continue;
      }

      const pinocchio::SE3 &oMf = data.oMf[fid];
      const pinocchio::SE3 oMv = oMf * vis.visual_placement;

      const Eigen::Vector3d txyz = oMv.translation();
      const Eigen::Quaterniond qrot(oMv.rotation());

      rec.log(vis.entity_path,
              rerun::Transform3D::from_translation_rotation(
                  {static_cast<float>(txyz.x()), static_cast<float>(txyz.y()),
                   static_cast<float>(txyz.z())},
                  rerun::Quaternion::from_xyzw(static_cast<float>(qrot.x()),
                                               static_cast<float>(qrot.y()),
                                               static_cast<float>(qrot.z()),
                                               static_cast<float>(qrot.w()))));
    }

    if (!visuals.empty()) {
      const auto debug_fid = visuals.front().parent_frame_id;
      if (debug_fid < data.oMf.size()) {
        const auto &pose = data.oMf[debug_fid];
        auto pts = onePoint(pose.translation());
        rec.log("robot/debug/frame0",
                rerun::Points3D(pts).with_radii(std::vector<float>{0.08f}));
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return 0;
}
