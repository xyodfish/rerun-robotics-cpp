#include "demo/trajectory_dispatcher.h"

#include <pinocchio/spatial/se3.hpp>

namespace demo {

TrajectoryDispatcher::TrajectoryDispatcher(rerun::RecordingStream recording)
    : recording_(std::move(recording)) {
}

void TrajectoryDispatcher::initWorldFrame() {
    recording_.set_time_sequence("frame", 0);
    recording_.log("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);
}

void TrajectoryDispatcher::publishStaticMeshes(const std::vector<VisualInstance>& visuals) {
    for (const auto& vis : visuals) {
        if (!vis.mesh) {
            continue;
        }

        auto verts = toRerunPositions(vis.mesh->vertices);
        auto normals = toRerunNormals(vis.mesh->normals);
        auto tris = toRerunTriangles(vis.mesh->triangles);

        recording_.log(
            vis.entityPath,
            rerun::Mesh3D(verts)
                .with_triangle_indices(tris)
                .with_vertex_normals(normals)
                .with_albedo_factor(rerun::components::AlbedoFactor(0xB4B4B4FF)));
    }
}

void TrajectoryDispatcher::publishVisualTransforms(
    int frame, const std::vector<VisualInstance>& visuals, const pinocchio::Data& data) {
    recording_.set_time_sequence("frame", frame);

    for (const auto& vis : visuals) {
        const pinocchio::FrameIndex fid = vis.parentFrameId;
        if (fid >= data.oMf.size()) {
            continue;
        }

        const pinocchio::SE3& oMf = data.oMf[fid];
        const pinocchio::SE3 oMv = oMf * vis.visualPlacement;

        const Eigen::Vector3d txyz = oMv.translation();
        const Eigen::Quaterniond qrot(oMv.rotation());

        recording_.log(
            vis.entityPath,
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
}

void TrajectoryDispatcher::publishDebugPoint(
    const std::vector<VisualInstance>& visuals, const pinocchio::Data& data) {
    if (visuals.empty()) {
        return;
    }

    const auto debugFid = visuals.front().parentFrameId;
    if (debugFid >= data.oMf.size()) {
        return;
    }

    const auto& pose = data.oMf[debugFid];
    std::vector<rerun::Position3D> pts;
    pts.emplace_back(
        static_cast<float>(pose.translation().x()),
        static_cast<float>(pose.translation().y()),
        static_cast<float>(pose.translation().z()));

    recording_.log("robot/debug/frame0", rerun::Points3D(pts).with_radii(std::vector<float>{0.08f}));
}

std::vector<rerun::Position3D> TrajectoryDispatcher::toRerunPositions(
    const std::vector<Eigen::Vector3f>& pts) {
    std::vector<rerun::Position3D> out;
    out.reserve(pts.size());
    for (const auto& p : pts) {
        out.emplace_back(p.x(), p.y(), p.z());
    }
    return out;
}

std::vector<rerun::components::Vector3D> TrajectoryDispatcher::toRerunNormals(
    const std::vector<Eigen::Vector3f>& normals) {
    std::vector<rerun::components::Vector3D> out;
    out.reserve(normals.size());
    for (const auto& n : normals) {
        out.emplace_back(n.x(), n.y(), n.z());
    }
    return out;
}

std::vector<rerun::components::TriangleIndices> TrajectoryDispatcher::toRerunTriangles(
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

} // namespace demo
