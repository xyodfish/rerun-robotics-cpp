#pragma once

#include "demo/robot_model_runtime.h"

#include <rerun.hpp>

namespace demo {

class TrajectoryDispatcher {
public:
    explicit TrajectoryDispatcher(rerun::RecordingStream recording);

    void initWorldFrame();
    void publishStaticMeshes(const std::vector<VisualInstance>& visuals);
    void publishVisualTransforms(
        int frame, const std::vector<VisualInstance>& visuals, const pinocchio::Data& data);
    void publishDebugPoint(const std::vector<VisualInstance>& visuals, const pinocchio::Data& data);

private:
    static std::vector<rerun::Position3D> toRerunPositions(const std::vector<Eigen::Vector3f>& pts);
    static std::vector<rerun::components::Vector3D> toRerunNormals(
        const std::vector<Eigen::Vector3f>& normals);
    static std::vector<rerun::components::TriangleIndices> toRerunTriangles(
        const std::vector<Eigen::Vector3i>& tris);

private:
    rerun::RecordingStream recording_;
};

} // namespace demo
