#include "testbed/obstacle_mode.hpp"

#include "testbed/sensor.hpp"

std::string obstacleModeName(ObstacleMode mode) {
    return mode == ObstacleMode::KnownGeometry ? "known" : "sensed";
}

ObstacleOptions obstacleOptions(ObstacleMode mode, const SceneEditor& edit_scene) {
    ObstacleOptions options;
    if (mode == ObstacleMode::KnownGeometry) {
        options.edit_scene = edit_scene;
    } else {
        options.source = ObstacleSource::PointCloud;
    }
    return options;
}

PointCloud senseObstacles(MotionPlanner& planner, const robots::TestRobot& robot,
                          const std::string& scene_xml, const SceneEditor& edit_scene,
                          std::uint32_t seed) {
    SimulatedDepthSensor sensor(scene_xml, robot.cameras, edit_scene, seed);
    sensor.setSceneQpos(planner.toSceneQpos(planner.startJoints()));
    PointCloud cloud = sensor.capture();
    planner.setPointCloud(cloud);
    return cloud;
}
