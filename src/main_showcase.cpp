#include <pinocchio/algorithm/joint-configuration.hpp>

#include <rerun.hpp>

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <future>
#include <fstream>
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
    pinocchio::JointIndex joint_id = 0;
    int q_idx = -1;
    double q0 = 0.0;
    double amp = 0.2;
    double freq = 0.2;
    double phase = 0.0;
};

static Eigen::Vector3f toVec3f(const aiVector3D& v) {
    return Eigen::Vector3f(v.x, v.y, v.z);
}

static std::string sanitizeName(const std::string& s) {
    std::string out = s;
    for (auto& c : out) {
        if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '/')) {
            c = '_';
        }
    }
    return out;
}

static std::string resolveMeshPath(
    const std::string& mesh_uri, const std::string& urdf_path, const std::string& mesh_root_dir) {
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
            return (fs::path(mesh_root_dir) / fs::path(rel).lexically_relative("meshes")).string();
        }
    }

    return (fs::path(urdf_path).parent_path() / p).string();
}

static MeshData loadMeshWithAssimp(const std::string& mesh_file, const Eigen::Vector3d& scale) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(
        mesh_file,
        aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_ImproveCacheLocality |
            aiProcess_SortByPType | aiProcess_GenSmoothNormals);

    if (!scene || !scene->HasMeshes()) {
        throw std::runtime_error("Failed to load mesh: " + mesh_file);
    }

    MeshData out;
    size_t vertex_offset = 0;

    for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];
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
            const aiFace& face = mesh->mFaces[f];
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

static std::string meshCacheKey(const std::string& mesh_file, const Eigen::Vector3d& scale);

static bool writeMeshCache(const std::string& cache_file, const MeshData& mesh) {
    std::ofstream os(cache_file, std::ios::binary | std::ios::trunc);
    if (!os.good()) {
        return false;
    }

    const uint32_t magic = 0x4D534831;
    const uint32_t version = 1;
    const uint64_t v_count = static_cast<uint64_t>(mesh.vertices.size());
    const uint64_t n_count = static_cast<uint64_t>(mesh.normals.size());
    const uint64_t t_count = static_cast<uint64_t>(mesh.triangles.size());

    os.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    os.write(reinterpret_cast<const char*>(&version), sizeof(version));
    os.write(reinterpret_cast<const char*>(&v_count), sizeof(v_count));
    os.write(reinterpret_cast<const char*>(&n_count), sizeof(n_count));
    os.write(reinterpret_cast<const char*>(&t_count), sizeof(t_count));

    if (v_count > 0) {
        os.write(
            reinterpret_cast<const char*>(mesh.vertices.data()),
            static_cast<std::streamsize>(v_count * sizeof(Eigen::Vector3f)));
    }
    if (n_count > 0) {
        os.write(
            reinterpret_cast<const char*>(mesh.normals.data()),
            static_cast<std::streamsize>(n_count * sizeof(Eigen::Vector3f)));
    }
    if (t_count > 0) {
        os.write(
            reinterpret_cast<const char*>(mesh.triangles.data()),
            static_cast<std::streamsize>(t_count * sizeof(Eigen::Vector3i)));
    }
    return os.good();
}

static bool readMeshCache(const std::string& cache_file, MeshData* out) {
    if (!out) {
        return false;
    }
    std::ifstream is(cache_file, std::ios::binary);
    if (!is.good()) {
        return false;
    }

    uint32_t magic = 0;
    uint32_t version = 0;
    uint64_t v_count = 0;
    uint64_t n_count = 0;
    uint64_t t_count = 0;

    is.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    is.read(reinterpret_cast<char*>(&version), sizeof(version));
    is.read(reinterpret_cast<char*>(&v_count), sizeof(v_count));
    is.read(reinterpret_cast<char*>(&n_count), sizeof(n_count));
    is.read(reinterpret_cast<char*>(&t_count), sizeof(t_count));

    if (!is.good() || magic != 0x4D534831 || version != 1) {
        return false;
    }

    MeshData mesh;
    mesh.vertices.resize(static_cast<size_t>(v_count));
    mesh.normals.resize(static_cast<size_t>(n_count));
    mesh.triangles.resize(static_cast<size_t>(t_count));

    if (v_count > 0) {
        is.read(
            reinterpret_cast<char*>(mesh.vertices.data()),
            static_cast<std::streamsize>(v_count * sizeof(Eigen::Vector3f)));
    }
    if (n_count > 0) {
        is.read(
            reinterpret_cast<char*>(mesh.normals.data()),
            static_cast<std::streamsize>(n_count * sizeof(Eigen::Vector3f)));
    }
    if (t_count > 0) {
        is.read(
            reinterpret_cast<char*>(mesh.triangles.data()),
            static_cast<std::streamsize>(t_count * sizeof(Eigen::Vector3i)));
    }
    if (!is.good() || mesh.vertices.empty() || mesh.triangles.empty()) {
        return false;
    }

    *out = std::move(mesh);
    return true;
}

static std::string meshCacheFilePath(const std::string& mesh_file, const Eigen::Vector3d& scale) {
    const std::string key = meshCacheKey(mesh_file, scale);
    const auto key_hash = std::hash<std::string>{}(key);
    const fs::path cache_dir = fs::path("/tmp") / "rerun_mesh_cache";
    return (cache_dir / (std::to_string(key_hash) + ".bin")).string();
}

static MeshData loadMeshWithCache(const std::string& mesh_file, const Eigen::Vector3d& scale) {
    const std::string cache_file = meshCacheFilePath(mesh_file, scale);
    const fs::path cache_path(cache_file);
    const fs::path mesh_path(mesh_file);

    try {
        fs::create_directories(cache_path.parent_path());
    } catch (...) {
    }

    bool cache_valid = false;
    try {
        if (fs::exists(cache_path) && fs::exists(mesh_path)) {
            cache_valid = fs::last_write_time(cache_path) >= fs::last_write_time(mesh_path);
        }
    } catch (...) {
        cache_valid = false;
    }

    if (cache_valid) {
        MeshData cached;
        if (readMeshCache(cache_file, &cached)) {
            return cached;
        }
    }

    MeshData loaded = loadMeshWithAssimp(mesh_file, scale);
    (void)writeMeshCache(cache_file, loaded);
    return loaded;
}

static std::string meshCacheKey(const std::string& mesh_file, const Eigen::Vector3d& scale) {
    const auto sx = static_cast<long long>(std::llround(scale.x() * 1e9));
    const auto sy = static_cast<long long>(std::llround(scale.y() * 1e9));
    const auto sz = static_cast<long long>(std::llround(scale.z() * 1e9));
    return mesh_file + "|" + std::to_string(sx) + "," + std::to_string(sy) + "," +
           std::to_string(sz);
}

static std::vector<VisualInstance> loadVisualsFromUrdf(
    const std::string& urdf_path, const std::string& package_root_dir,
    const std::string& mesh_root_dir, const pinocchio::Model& model) {
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
        const auto& frame = model.frames[fid];
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
            const auto& visual = link_visuals[vis_idx];
            if (!visual || !visual->geometry || visual->geometry->type != urdf::Geometry::MESH) {
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

            const std::string mesh_path = resolveMeshPath(mesh->filename, urdf_path, mesh_root_dir);
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

            Eigen::Vector3d txyz(
                visual->origin.position.x,
                visual->origin.position.y,
                visual->origin.position.z);
            Eigen::Quaterniond qrot(
                visual->origin.rotation.w,
                visual->origin.rotation.x,
                visual->origin.rotation.y,
                visual->origin.rotation.z);
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
        auto& front = in_flight.front();
        auto mesh = front.second.get();
        loaded_meshes[front.first] = std::make_shared<MeshData>(std::move(mesh));
        in_flight.pop_front();
    };

    for (size_t i = 0; i < unique_meshes.size(); ++i) {
        const auto mesh_path = unique_meshes[i].mesh_path;
        const auto scale = unique_meshes[i].scale;

        in_flight.emplace_back(i, std::async(std::launch::async, [mesh_path, scale]() {
                                   return loadMeshWithCache(mesh_path, scale);
                               }));

        if (in_flight.size() >= max_workers) {
            drainOne();
        }
    }

    while (!in_flight.empty()) {
        drainOne();
    }

    for (const auto& prepared : prepared_visuals) {
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
    const auto prep_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    const auto load_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    std::cout << "[INFO] visual prep: " << prep_ms << " ms, mesh load: " << load_ms
              << " ms, visuals: " << visuals.size() << ", unique meshes: " << unique_meshes.size()
              << std::endl;

    return visuals;
}

static std::vector<rerun::Position3D> toRerunPositions(const std::vector<Eigen::Vector3f>& pts) {
    std::vector<rerun::Position3D> out;
    out.reserve(pts.size());
    for (const auto& p : pts) {
        out.emplace_back(p.x(), p.y(), p.z());
    }
    return out;
}

static std::vector<rerun::components::Vector3D> toRerunNormals(
    const std::vector<Eigen::Vector3f>& normals) {
    std::vector<rerun::components::Vector3D> out;
    out.reserve(normals.size());
    for (const auto& n : normals) {
        out.emplace_back(n.x(), n.y(), n.z());
    }
    return out;
}

static std::vector<rerun::components::TriangleIndices> toRerunTriangles(
    const std::vector<Eigen::Vector3i>& tris) {
    std::vector<rerun::components::TriangleIndices> out;
    out.reserve(tris.size());
    for (const auto& t : tris) {
        out.emplace_back(
            static_cast<uint32_t>(t.x()),
            static_cast<uint32_t>(t.y()),
            static_cast<uint32_t>(t.z()));
    }
    return out;
}

static rerun::Position3D toRerunPosition(const Eigen::Vector3d& p) {
    return rerun::Position3D(
        static_cast<float>(p.x()),
        static_cast<float>(p.y()),
        static_cast<float>(p.z()));
}

static rerun::components::Vector3D toRerunVector(const Eigen::Vector3d& v) {
    return rerun::components::Vector3D(
        static_cast<float>(v.x()),
        static_cast<float>(v.y()),
        static_cast<float>(v.z()));
}

static rerun::components::LineStrip3D toLineStrip(const std::vector<Eigen::Vector3d>& points) {
    std::vector<rerun::datatypes::Vec3D> rr_points;
    rr_points.reserve(points.size());
    for (const auto& p : points) {
        rr_points.emplace_back(
            static_cast<float>(p.x()),
            static_cast<float>(p.y()),
            static_cast<float>(p.z()));
    }
    return rerun::components::LineStrip3D(std::move(rr_points));
}

static rerun::components::LineStrip3D toLineStrip(const std::deque<Eigen::Vector3d>& points) {
    std::vector<rerun::datatypes::Vec3D> rr_points;
    rr_points.reserve(points.size());
    for (const auto& p : points) {
        rr_points.emplace_back(
            static_cast<float>(p.x()),
            static_cast<float>(p.y()),
            static_cast<float>(p.z()));
    }
    return rerun::components::LineStrip3D(std::move(rr_points));
}

static rerun::components::Color errorToColor(double err) {
    if (err < 0.06) {
        return rerun::components::Color(50, 205, 50);
    }
    if (err < 0.14) {
        return rerun::components::Color(255, 215, 0);
    }
    return rerun::components::Color(255, 69, 0);
}

static Eigen::Vector3d makeTargetPosition(double t) {
    return Eigen::Vector3d(
        0.55 + 0.12 * std::cos(1.3 * t),
        -0.15 + 0.18 * std::sin(0.9 * t),
        0.85 + 0.10 * std::sin(1.7 * t + 0.3));
}

static std::vector<rerun::components::Color> makeJointPalette() {
    return {
        rerun::components::Color(255, 99, 71),
        rerun::components::Color(255, 165, 0),
        rerun::components::Color(255, 215, 0),
        rerun::components::Color(50, 205, 50),
        rerun::components::Color(64, 224, 208),
        rerun::components::Color(70, 130, 180),
        rerun::components::Color(186, 85, 211),
    };
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

    std::cout << "[INFO] model nq=" << model.nq << ", nv=" << model.nv << std::endl;

    auto visuals = loadVisualsFromUrdf(urdf_path, package_root_dir, mesh_root_dir, model);
    std::cout << "[INFO] loaded visuals: " << visuals.size() << std::endl;

    auto rec = rerun::RecordingStream("galbot_showcase");
    rec.spawn().exit_on_failure();

    // Keep all spatial data on the same timeline so temporal views always have geometry.
    rec.set_time_sequence("frame", 0);
    rec.log("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

    // mesh 只发一次（frame=0），后续帧只更新 transform
    for (const auto& vis : visuals) {
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
        "left_arm_joint1",
        "left_arm_joint2",
        "left_arm_joint3",
        "left_arm_joint4",
        "left_arm_joint5",
        "left_arm_joint6",
        "left_arm_joint7"};

    std::vector<JointMotion> motions;
    motions.reserve(joint_names.size());

    for (size_t i = 0; i < joint_names.size(); ++i) {
        const auto jid = model.getJointId(joint_names[i]);
        if (jid == 0) {
            std::cerr << "[WARN] joint not found: " << joint_names[i] << std::endl;
            continue;
        }

        const auto& jmodel = model.joints[jid];
        if (jmodel.nq() != 1) {
            std::cerr << "[WARN] joint nq != 1, skip: " << joint_names[i] << std::endl;
            continue;
        }

        JointMotion jm;
        jm.name = joint_names[i];
        jm.joint_id = jid;
        jm.q_idx = jmodel.idx_q();
        jm.q0 = q[jm.q_idx];
        jm.amp = 0.18 + 0.02 * static_cast<double>(i);
        jm.freq = 0.18 + 0.015 * static_cast<double>(i);
        jm.phase = 0.35 * static_cast<double>(i);

        motions.push_back(jm);

        std::cout << "[INFO] animate joint " << jm.name << " q_idx=" << jm.q_idx << std::endl;
    }

    const auto joint_palette = makeJointPalette();
    rec.log_static(
        "plots/ee/error_norm",
        rerun::SeriesLines()
            .with_names("ee_error_norm")
            .with_colors(
                std::vector<rerun::components::Color>{rerun::components::Color(255, 99, 71)}));
    rec.log_static(
        "plots/ee/speed",
        rerun::SeriesLines()
            .with_names("ee_speed")
            .with_colors(
                std::vector<rerun::components::Color>{rerun::components::Color(70, 130, 180)}));

    for (size_t i = 0; i < motions.size(); ++i) {
        const auto color = joint_palette[i % joint_palette.size()];
        const std::string path = "plots/joints/" + motions[i].name;
        rec.log_static(
            path,
            rerun::SeriesLines()
                .with_names(motions[i].name.c_str())
                .with_colors(std::vector<rerun::components::Color>{color}));
    }

    std::deque<Eigen::Vector3d> ee_trail;
    std::deque<Eigen::Vector3d> target_trail;
    const size_t trail_max_size = 360;

    Eigen::Vector3d prev_ee_pos = Eigen::Vector3d::Zero();
    bool has_prev_ee = false;

    for (int frame = 0;; ++frame) {
        const double dt = 0.02;
        const double t = dt * static_cast<double>(frame);
        rec.set_time_sequence("frame", frame);

        for (const auto& jm : motions) {
            q[jm.q_idx] = jm.q0 + jm.amp * std::sin(2.0 * M_PI * jm.freq * t + jm.phase);
        }

        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);

        for (const auto& vis : visuals) {
            const pinocchio::FrameIndex fid = vis.parent_frame_id;
            if (fid >= data.oMf.size()) {
                continue;
            }

            const pinocchio::SE3& oMf = data.oMf[fid];
            const pinocchio::SE3 oMv = oMf * vis.visual_placement;

            const Eigen::Vector3d txyz = oMv.translation();
            const Eigen::Quaterniond qrot(oMv.rotation());

            rec.log(
                vis.entity_path,
                rerun::Transform3D::from_translation_rotation(
                    {static_cast<float>(txyz.x()),
                     static_cast<float>(txyz.y()),
                     static_cast<float>(txyz.z())},
                    rerun::Quaternion::from_xyzw(
                        static_cast<float>(qrot.x()),
                        static_cast<float>(qrot.y()),
                        static_cast<float>(qrot.z()),
                        static_cast<float>(qrot.w()))));
        }

        std::vector<Eigen::Vector3d> joint_positions;
        joint_positions.reserve(motions.size());
        for (const auto& jm : motions) {
            if (jm.joint_id < data.oMi.size()) {
                joint_positions.push_back(data.oMi[jm.joint_id].translation());
            }
        }

        Eigen::Vector3d ee_pos = Eigen::Vector3d::Zero();
        if (!joint_positions.empty()) {
            ee_pos = joint_positions.back();
        }

        if (!has_prev_ee) {
            prev_ee_pos = ee_pos;
            has_prev_ee = true;
        }

        const Eigen::Vector3d ee_vel = (ee_pos - prev_ee_pos) / dt;
        prev_ee_pos = ee_pos;

        const Eigen::Vector3d target_pos = makeTargetPosition(t);
        const Eigen::Vector3d err_vec = target_pos - ee_pos;
        const double err_norm = err_vec.norm();
        const double speed = ee_vel.norm();

        ee_trail.push_back(ee_pos);
        target_trail.push_back(target_pos);
        if (ee_trail.size() > trail_max_size) {
            ee_trail.pop_front();
        }
        if (target_trail.size() > trail_max_size) {
            target_trail.pop_front();
        }

        if (!joint_positions.empty()) {
            std::vector<rerun::Position3D> rr_joint_points;
            rr_joint_points.reserve(joint_positions.size());
            std::vector<rerun::components::Color> rr_joint_colors;
            rr_joint_colors.reserve(joint_positions.size());
            std::vector<float> rr_joint_radii;
            rr_joint_radii.reserve(joint_positions.size());

            for (size_t i = 0; i < joint_positions.size(); ++i) {
                rr_joint_points.push_back(toRerunPosition(joint_positions[i]));
                rr_joint_colors.push_back(joint_palette[i % joint_palette.size()]);
                rr_joint_radii.push_back(0.018f);
            }

            rec.log(
                "robot/debug/joint_chain/points",
                rerun::Points3D(rr_joint_points)
                    .with_colors(rr_joint_colors)
                    .with_radii(rr_joint_radii));

            rec.log(
                "robot/debug/joint_chain/line",
                rerun::LineStrips3D(
                    std::vector<rerun::components::LineStrip3D>{toLineStrip(joint_positions)})
                    .with_colors(std::vector<rerun::components::Color>{
                        rerun::components::Color(0, 220, 255)})
                    .with_radii(std::vector<float>{0.006f}));
        }

        rec.log(
            "robot/debug/end_effector",
            rerun::Points3D(std::vector<rerun::Position3D>{toRerunPosition(ee_pos)})
                .with_colors(
                    std::vector<rerun::components::Color>{rerun::components::Color(0, 255, 127)})
                .with_radii(std::vector<float>{0.03f}));

        rec.log(
            "robot/debug/target",
            rerun::Points3D(std::vector<rerun::Position3D>{toRerunPosition(target_pos)})
                .with_colors(
                    std::vector<rerun::components::Color>{rerun::components::Color(255, 69, 0)})
                .with_radii(std::vector<float>{0.03f}));

        rec.log(
            "robot/debug/ee_error",
            rerun::LineStrips3D(std::vector<rerun::components::LineStrip3D>{
                                    toLineStrip(std::vector<Eigen::Vector3d>{ee_pos, target_pos})})
                .with_colors(std::vector<rerun::components::Color>{errorToColor(err_norm)})
                .with_radii(std::vector<float>{0.01f}));

        if (ee_trail.size() >= 2) {
            rec.log(
                "robot/debug/ee_trail",
                rerun::LineStrips3D(
                    std::vector<rerun::components::LineStrip3D>{toLineStrip(ee_trail)})
                    .with_colors(std::vector<rerun::components::Color>{
                        rerun::components::Color(255, 140, 0)})
                    .with_radii(std::vector<float>{0.007f}));
        }

        if (target_trail.size() >= 2) {
            rec.log(
                "robot/debug/target_trail",
                rerun::LineStrips3D(
                    std::vector<rerun::components::LineStrip3D>{toLineStrip(target_trail)})
                    .with_colors(std::vector<rerun::components::Color>{
                        rerun::components::Color(220, 20, 60)})
                    .with_radii(std::vector<float>{0.004f}));
        }

        rec.log(
            "robot/debug/ee_velocity",
            rerun::Arrows3D::from_vectors(
                std::vector<rerun::components::Vector3D>{toRerunVector(ee_vel * 0.08)})
                .with_origins(std::vector<rerun::Position3D>{toRerunPosition(ee_pos)})
                .with_colors(
                    std::vector<rerun::components::Color>{rerun::components::Color(65, 105, 225)})
                .with_radii(std::vector<float>{0.012f}));

        for (const auto& jm : motions) {
            const std::string path = "plots/joints/" + jm.name;
            rec.log(path, rerun::Scalars(q[jm.q_idx]));
        }
        rec.log("plots/ee/error_norm", rerun::Scalars(err_norm));
        rec.log("plots/ee/speed", rerun::Scalars(speed));

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
}
