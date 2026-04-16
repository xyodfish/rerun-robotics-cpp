#include "demo/robot_model_runtime.h"
#include "demo/trajectory_dispatcher.h"

#include <rerun.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

int main() {
    const std::string urdfPath =
        "/data/config/galbot_description/galbot_one_golf_description/"
        "galbot_one_golf.urdf";
    const std::string packageRootDir = "/data/config/galbot_description";

    demo::RobotModelRuntime runtime(urdfPath, packageRootDir);
    runtime.load();

    std::cout << "[INFO] model nq=" << runtime.model().nq << ", nv=" << runtime.model().nv << std::endl;
    std::cout << "[INFO] urdfdom world models=" << runtime.worldModelCount()
              << ", sensor groups=" << runtime.sensorGroupCount() << std::endl;
    std::cout << "[INFO] loaded visuals: " << runtime.visuals().size() << std::endl;

    auto rec = rerun::RecordingStream("galbot_mesh_anim");
    rec.spawn().exit_on_failure();

    demo::TrajectoryDispatcher dispatcher(std::move(rec));
    dispatcher.initWorldFrame();
    dispatcher.publishStaticMeshes(runtime.visuals());

    const std::vector<std::string> jointNames = {
        "left_arm_joint1",
        "left_arm_joint2",
        "left_arm_joint3",
        "left_arm_joint4",
        "left_arm_joint5",
        "left_arm_joint6",
        "left_arm_joint7"};

    const auto motions = runtime.createSineMotions(jointNames);

    for (int frame = 0;; ++frame) {
        const double t = 0.02 * static_cast<double>(frame);

        runtime.applySineMotions(motions, t);
        runtime.updateKinematics();

        dispatcher.publishVisualTransforms(frame, runtime.visuals(), runtime.data());
        dispatcher.publishDebugPoint(runtime.visuals(), runtime.data());

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 0;
}
