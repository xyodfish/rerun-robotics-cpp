#pragma once

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <urdf_model/model.h>
#include <urdf_sensor/sensor.h>
#include <urdf_world/world.h>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace demo {

struct MeshData {
    std::vector<Eigen::Vector3f> vertices;
    std::vector<Eigen::Vector3f> normals;
    std::vector<Eigen::Vector3i> triangles;
};

struct VisualInstance {
    std::string entityPath;
    pinocchio::FrameIndex parentFrameId{};
    std::string parentFrameName;
    pinocchio::SE3 visualPlacement;
    Eigen::Vector3d scale = Eigen::Vector3d::Ones();
    std::shared_ptr<const MeshData> mesh;
};

struct JointMotion {
    std::string name;
    int qIdx = -1;
    double q0 = 0.0;
    double amp = 0.2;
    double freq = 0.2;
    double phase = 0.0;
};

class RobotModelRuntime {
public:
    RobotModelRuntime(std::string urdfPath, std::string packageRootDir);

    void load();

    const pinocchio::Model& model() const;
    pinocchio::Data& data();
    const pinocchio::Data& data() const;

    Eigen::VectorXd& configuration();
    const Eigen::VectorXd& configuration() const;

    const std::vector<VisualInstance>& visuals() const;

    size_t worldModelCount() const;
    size_t sensorGroupCount() const;

    std::vector<JointMotion> createSineMotions(const std::vector<std::string>& jointNames) const;
    void applySineMotions(const std::vector<JointMotion>& motions, double t);
    void updateKinematics();

private:
    struct UrdfRobotModel {
        urdf::ModelInterfaceSharedPtr urdfModel;
        urdf::World world;
        std::unordered_map<std::string, std::vector<urdf::Sensor>> sensorsByLinkName;
        std::unordered_map<std::string, pinocchio::FrameIndex> bodyFrameByLinkName;
    };

    static std::string sanitizeName(const std::string& s);

    static std::string resolveMeshPath(
        const std::string& meshUri, const std::string& urdfPath,
        const std::string& packageRootDir);

    static std::string meshCacheKey(const std::string& meshFile, const Eigen::Vector3d& scale);
    static std::string meshCacheFilePath(const std::string& meshFile, const Eigen::Vector3d& scale);

    static MeshData loadMeshWithAssimp(const std::string& meshFile, const Eigen::Vector3d& scale);
    static MeshData loadMeshWithCache(const std::string& meshFile, const Eigen::Vector3d& scale);

    static bool writeMeshCache(const std::string& cacheFile, const MeshData& mesh);
    static bool readMeshCache(const std::string& cacheFile, MeshData* out);

    UrdfRobotModel loadUrdfRobotModel() const;
    std::vector<VisualInstance> loadVisualsFromUrdf(const UrdfRobotModel& robotModel) const;

private:
    std::string urdfPath_;
    std::string packageRootDir_;

    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;
    Eigen::VectorXd q_;

    UrdfRobotModel urdfRobotModel_;
    std::vector<VisualInstance> visuals_;
};

} // namespace demo
