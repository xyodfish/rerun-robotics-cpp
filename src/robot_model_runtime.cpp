#include "demo/robot_model_runtime.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <urdf_parser/urdf_parser.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace fs = std::filesystem;

namespace demo {

static Eigen::Vector3f toVec3f(const aiVector3D& v) {
    return Eigen::Vector3f(v.x, v.y, v.z);
}

RobotModelRuntime::RobotModelRuntime(std::string urdfPath, std::string packageRootDir)
    : urdfPath_(std::move(urdfPath)), packageRootDir_(std::move(packageRootDir)) {
}

void RobotModelRuntime::load() {
    pinocchio::urdf::buildModel(urdfPath_, model_);
    data_ = std::make_unique<pinocchio::Data>(model_);
    q_ = pinocchio::neutral(model_);

    urdfRobotModel_ = loadUrdfRobotModel();
    visuals_ = loadVisualsFromUrdf(urdfRobotModel_);
}

const pinocchio::Model& RobotModelRuntime::model() const {
    return model_;
}

pinocchio::Data& RobotModelRuntime::data() {
    return *data_;
}

const pinocchio::Data& RobotModelRuntime::data() const {
    return *data_;
}

Eigen::VectorXd& RobotModelRuntime::configuration() {
    return q_;
}

const Eigen::VectorXd& RobotModelRuntime::configuration() const {
    return q_;
}

const std::vector<VisualInstance>& RobotModelRuntime::visuals() const {
    return visuals_;
}

size_t RobotModelRuntime::worldModelCount() const {
    return urdfRobotModel_.world.models.size();
}

size_t RobotModelRuntime::sensorGroupCount() const {
    return urdfRobotModel_.sensorsByLinkName.size();
}

std::vector<JointMotion> RobotModelRuntime::createSineMotions(
    const std::vector<std::string>& jointNames) const {
    std::vector<JointMotion> motions;
    motions.reserve(jointNames.size());

    const Eigen::VectorXd& q = configuration();

    for (size_t i = 0; i < jointNames.size(); ++i) {
        const auto jid = model_.getJointId(jointNames[i]);
        if (jid == 0) {
            std::cerr << "[WARN] joint not found: " << jointNames[i] << std::endl;
            continue;
        }

        const auto& jmodel = model_.joints[jid];
        if (jmodel.nq() != 1) {
            std::cerr << "[WARN] joint nq != 1, skip: " << jointNames[i] << std::endl;
            continue;
        }

        JointMotion jm;
        jm.name = jointNames[i];
        jm.qIdx = jmodel.idx_q();
        jm.q0 = q[jm.qIdx];
        jm.amp = 0.18 + 0.02 * static_cast<double>(i);
        jm.freq = 0.18 + 0.015 * static_cast<double>(i);
        jm.phase = 0.35 * static_cast<double>(i);
        motions.push_back(jm);

        std::cout << "[INFO] animate joint " << jm.name << " q_idx=" << jm.qIdx << std::endl;
    }

    return motions;
}

void RobotModelRuntime::applySineMotions(const std::vector<JointMotion>& motions, double t) {
    for (const auto& jm : motions) {
        q_[jm.qIdx] = jm.q0 + jm.amp * std::sin(2.0 * M_PI * jm.freq * t + jm.phase);
    }
}

void RobotModelRuntime::updateKinematics() {
    pinocchio::forwardKinematics(model_, *data_, q_);
    pinocchio::updateFramePlacements(model_, *data_);
}

std::string RobotModelRuntime::sanitizeName(const std::string& s) {
    std::string out = s;
    for (auto& c : out) {
        if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '/')) {
            c = '_';
        }
    }
    return out;
}

std::string RobotModelRuntime::resolveMeshPath(
    const std::string& meshUri, const std::string& urdfPath,
    const std::string& packageRootDir) {
    const std::string filePrefix = "file://";
    if (meshUri.rfind(filePrefix, 0) == 0) {
        return fs::path(meshUri.substr(filePrefix.size())).lexically_normal().string();
    }

    fs::path p(meshUri);
    if (p.is_absolute()) {
        return p.lexically_normal().string();
    }

    const std::string prefix = "package://";
    if (meshUri.rfind(prefix, 0) == 0) {
        auto rest = meshUri.substr(prefix.size());
        auto slashPos = rest.find('/');
        if (slashPos != std::string::npos) {
            const auto packageName = rest.substr(0, slashPos);
            const auto rel = rest.substr(slashPos + 1);
            return (fs::path(packageRootDir) / packageName / rel).lexically_normal().string();
        }
    }

    return (fs::path(urdfPath).parent_path() / p).lexically_normal().string();
}

RobotModelRuntime::UrdfRobotModel RobotModelRuntime::loadUrdfRobotModel() const {
    UrdfRobotModel out;
    out.urdfModel = urdf::parseURDFFile(urdfPath_);
    if (!out.urdfModel) {
        throw std::runtime_error("Failed to parse URDF with urdfdom_model: " + urdfPath_);
    }

    out.world.clear();
    out.world.name = "robot_world";
    urdf::Entity entity;
    entity.model = out.urdfModel;
    out.world.models.push_back(entity);

    out.bodyFrameByLinkName.reserve(model_.frames.size());
    for (size_t fid = 0; fid < model_.frames.size(); ++fid) {
        const auto& frame = model_.frames[fid];
        if (frame.type != pinocchio::BODY) {
            continue;
        }
        out.bodyFrameByLinkName.emplace(frame.name, static_cast<pinocchio::FrameIndex>(fid));
    }

    return out;
}

MeshData RobotModelRuntime::loadMeshWithAssimp(const std::string& meshFile, const Eigen::Vector3d& scale) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(meshFile, aiProcess_Triangulate);

    if (!scene || !scene->HasMeshes()) {
        throw std::runtime_error("Failed to load mesh: " + meshFile);
    }

    MeshData out;
    size_t totalVertices = 0;
    size_t totalTriangles = 0;

    for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];
        if (!mesh || !mesh->HasPositions()) {
            continue;
        }
        totalVertices += mesh->mNumVertices;
        for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
            if (mesh->mFaces[f].mNumIndices == 3) {
                ++totalTriangles;
            }
        }
    }

    out.vertices.reserve(totalVertices);
    out.normals.reserve(totalVertices);
    out.triangles.reserve(totalTriangles);

    size_t vertexOffset = 0;
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
            }
            out.normals.push_back(n);
        }

        for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
            const aiFace& face = mesh->mFaces[f];
            if (face.mNumIndices != 3) {
                continue;
            }
            out.triangles.emplace_back(
                static_cast<int>(vertexOffset + face.mIndices[0]),
                static_cast<int>(vertexOffset + face.mIndices[1]),
                static_cast<int>(vertexOffset + face.mIndices[2]));
        }

        vertexOffset += mesh->mNumVertices;
    }

    if (out.vertices.empty() || out.triangles.empty()) {
        throw std::runtime_error("Mesh has no valid triangles: " + meshFile);
    }

    return out;
}

bool RobotModelRuntime::writeMeshCache(const std::string& cacheFile, const MeshData& mesh) {
    std::ofstream os(cacheFile, std::ios::binary | std::ios::trunc);
    if (!os.good()) {
        return false;
    }

    const uint32_t magic = 0x4D534831;
    const uint32_t version = 1;
    const uint64_t vCount = static_cast<uint64_t>(mesh.vertices.size());
    const uint64_t nCount = static_cast<uint64_t>(mesh.normals.size());
    const uint64_t tCount = static_cast<uint64_t>(mesh.triangles.size());

    os.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    os.write(reinterpret_cast<const char*>(&version), sizeof(version));
    os.write(reinterpret_cast<const char*>(&vCount), sizeof(vCount));
    os.write(reinterpret_cast<const char*>(&nCount), sizeof(nCount));
    os.write(reinterpret_cast<const char*>(&tCount), sizeof(tCount));

    if (vCount > 0) {
        os.write(reinterpret_cast<const char*>(mesh.vertices.data()), static_cast<std::streamsize>(vCount * sizeof(Eigen::Vector3f)));
    }
    if (nCount > 0) {
        os.write(reinterpret_cast<const char*>(mesh.normals.data()), static_cast<std::streamsize>(nCount * sizeof(Eigen::Vector3f)));
    }
    if (tCount > 0) {
        os.write(reinterpret_cast<const char*>(mesh.triangles.data()), static_cast<std::streamsize>(tCount * sizeof(Eigen::Vector3i)));
    }

    return os.good();
}

bool RobotModelRuntime::readMeshCache(const std::string& cacheFile, MeshData* out) {
    if (!out) {
        return false;
    }

    std::ifstream is(cacheFile, std::ios::binary);
    if (!is.good()) {
        return false;
    }

    uint32_t magic = 0;
    uint32_t version = 0;
    uint64_t vCount = 0;
    uint64_t nCount = 0;
    uint64_t tCount = 0;

    is.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    is.read(reinterpret_cast<char*>(&version), sizeof(version));
    is.read(reinterpret_cast<char*>(&vCount), sizeof(vCount));
    is.read(reinterpret_cast<char*>(&nCount), sizeof(nCount));
    is.read(reinterpret_cast<char*>(&tCount), sizeof(tCount));

    if (!is.good() || magic != 0x4D534831 || version != 1) {
        return false;
    }

    MeshData mesh;
    mesh.vertices.resize(static_cast<size_t>(vCount));
    mesh.normals.resize(static_cast<size_t>(nCount));
    mesh.triangles.resize(static_cast<size_t>(tCount));

    if (vCount > 0) {
        is.read(reinterpret_cast<char*>(mesh.vertices.data()), static_cast<std::streamsize>(vCount * sizeof(Eigen::Vector3f)));
    }
    if (nCount > 0) {
        is.read(reinterpret_cast<char*>(mesh.normals.data()), static_cast<std::streamsize>(nCount * sizeof(Eigen::Vector3f)));
    }
    if (tCount > 0) {
        is.read(reinterpret_cast<char*>(mesh.triangles.data()), static_cast<std::streamsize>(tCount * sizeof(Eigen::Vector3i)));
    }

    if (!is.good() || mesh.vertices.empty() || mesh.triangles.empty()) {
        return false;
    }

    *out = std::move(mesh);
    return true;
}

std::string RobotModelRuntime::meshCacheKey(const std::string& meshFile, const Eigen::Vector3d& scale) {
    const auto sx = static_cast<long long>(std::llround(scale.x() * 1e9));
    const auto sy = static_cast<long long>(std::llround(scale.y() * 1e9));
    const auto sz = static_cast<long long>(std::llround(scale.z() * 1e9));
    return meshFile + "|" + std::to_string(sx) + "," + std::to_string(sy) + "," + std::to_string(sz);
}

std::string RobotModelRuntime::meshCacheFilePath(const std::string& meshFile, const Eigen::Vector3d& scale) {
    const std::string key = meshCacheKey(meshFile, scale);
    const auto keyHash = std::hash<std::string>{}(key);
    const fs::path cacheDir = fs::path("/tmp") / "rerun_mesh_cache";
    return (cacheDir / (std::to_string(keyHash) + ".bin")).string();
}

MeshData RobotModelRuntime::loadMeshWithCache(const std::string& meshFile, const Eigen::Vector3d& scale) {
    const std::string cacheFile = meshCacheFilePath(meshFile, scale);
    const fs::path cachePath(cacheFile);
    const fs::path meshPath(meshFile);

    try {
        fs::create_directories(cachePath.parent_path());
    } catch (...) {
    }

    bool cacheValid = false;
    try {
        if (fs::exists(cachePath) && fs::exists(meshPath)) {
            cacheValid = fs::last_write_time(cachePath) >= fs::last_write_time(meshPath);
        }
    } catch (...) {
        cacheValid = false;
    }

    if (cacheValid) {
        MeshData cached;
        if (readMeshCache(cacheFile, &cached)) {
            return cached;
        }
    }

    MeshData loaded = loadMeshWithAssimp(meshFile, scale);
    (void)writeMeshCache(cacheFile, loaded);
    return loaded;
}

std::vector<VisualInstance> RobotModelRuntime::loadVisualsFromUrdf(const UrdfRobotModel& robotModel) const {
    const auto t0 = std::chrono::steady_clock::now();

    struct PreparedVisual {
        std::string entityPath;
        pinocchio::FrameIndex parentFrameId{};
        std::string parentFrameName;
        pinocchio::SE3 visualPlacement;
        Eigen::Vector3d scale = Eigen::Vector3d::Ones();
        size_t meshIdx = 0;
    };

    struct MeshRequest {
        std::string meshPath;
        Eigen::Vector3d scale = Eigen::Vector3d::Ones();
    };

    std::unordered_map<std::string, size_t> meshKeyToIdx;
    std::vector<MeshRequest> uniqueMeshes;
    uniqueMeshes.reserve(64);

    std::vector<PreparedVisual> preparedVisuals;
    preparedVisuals.reserve(64);

    std::vector<VisualInstance> visuals;
    visuals.reserve(64);

    for (const auto& item : robotModel.urdfModel->links_) {
        const auto& frameIt = robotModel.bodyFrameByLinkName.find(item.first);
        if (frameIt == robotModel.bodyFrameByLinkName.end() || !item.second) {
            continue;
        }

        const auto& link = item.second;
        const pinocchio::FrameIndex parentFrameId = frameIt->second;

        std::vector<urdf::VisualSharedPtr> linkVisuals;
        if (!link->visual_array.empty()) {
            linkVisuals = link->visual_array;
        } else if (link->visual) {
            linkVisuals.push_back(link->visual);
        }

        for (size_t visIdx = 0; visIdx < linkVisuals.size(); ++visIdx) {
            const auto& visual = linkVisuals[visIdx];
            if (!visual || !visual->geometry || visual->geometry->type != urdf::Geometry::MESH) {
                continue;
            }

            const auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(visual->geometry);
            if (!mesh || mesh->filename.empty()) {
                continue;
            }

            Eigen::Vector3d meshScale = Eigen::Vector3d::Ones();
            meshScale.x() = mesh->scale.x;
            meshScale.y() = mesh->scale.y;
            meshScale.z() = mesh->scale.z;
            if ((meshScale.array() == 0.0).all()) {
                meshScale = Eigen::Vector3d::Ones();
            }

            const std::string meshPath = resolveMeshPath(mesh->filename, urdfPath_, packageRootDir_);
            const std::string key = meshCacheKey(meshPath, meshScale);

            size_t meshIdx = 0;
            const auto it = meshKeyToIdx.find(key);
            if (it == meshKeyToIdx.end()) {
                meshIdx = uniqueMeshes.size();
                uniqueMeshes.push_back(MeshRequest{meshPath, meshScale});
                meshKeyToIdx.emplace(key, meshIdx);
            } else {
                meshIdx = it->second;
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
            prepared.entityPath = "robot/visuals/" + sanitizeName(item.first + "_" + std::to_string(visIdx));
            prepared.parentFrameId = parentFrameId;
            prepared.parentFrameName = item.first;
            prepared.visualPlacement = pinocchio::SE3(qrot.toRotationMatrix(), txyz);
            prepared.scale = meshScale;
            prepared.meshIdx = meshIdx;
            preparedVisuals.push_back(std::move(prepared));
        }
    }

    const auto t1 = std::chrono::steady_clock::now();

    std::vector<std::shared_ptr<const MeshData>> loadedMeshes;
    loadedMeshes.resize(uniqueMeshes.size());

    const size_t hw = std::max<size_t>(1, static_cast<size_t>(std::thread::hardware_concurrency()));
    const size_t maxWorkers = std::max<size_t>(1, std::min<size_t>(hw, uniqueMeshes.size()));
    std::deque<std::pair<size_t, std::future<MeshData>>> inFlight;

    auto drainOne = [&]() {
        auto& front = inFlight.front();
        auto mesh = front.second.get();
        loadedMeshes[front.first] = std::make_shared<MeshData>(std::move(mesh));
        inFlight.pop_front();
    };

    for (size_t i = 0; i < uniqueMeshes.size(); ++i) {
        const auto meshPath = uniqueMeshes[i].meshPath;
        const auto scale = uniqueMeshes[i].scale;

        if (!fs::exists(meshPath)) {
            throw std::runtime_error("Mesh file does not exist: " + meshPath);
        }

        inFlight.emplace_back(i, std::async(std::launch::async, [meshPath, scale]() {
            return loadMeshWithCache(meshPath, scale);
        }));

        if (inFlight.size() >= maxWorkers) {
            drainOne();
        }
    }

    while (!inFlight.empty()) {
        drainOne();
    }

    for (const auto& prepared : preparedVisuals) {
        VisualInstance vis;
        vis.entityPath = prepared.entityPath;
        vis.parentFrameId = prepared.parentFrameId;
        vis.parentFrameName = prepared.parentFrameName;
        vis.visualPlacement = prepared.visualPlacement;
        vis.scale = prepared.scale;
        vis.mesh = loadedMeshes[prepared.meshIdx];
        visuals.push_back(std::move(vis));
    }

    const auto t2 = std::chrono::steady_clock::now();
    const auto prepMs = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    const auto loadMs = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

    std::cout << "[INFO] visual prep: " << prepMs << " ms, mesh load: " << loadMs
              << " ms, visuals: " << visuals.size() << ", unique meshes: " << uniqueMeshes.size() << std::endl;

    return visuals;
}

} // namespace demo
