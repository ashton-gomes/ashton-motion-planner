#include "testbed/sensor.hpp"

#include <mujoco/mujoco.h>

#include <cmath>
#include <random>
#include <stdexcept>

namespace {

constexpr double kPi = 3.14159265358979323846;

void validate(const DepthCameraSettings& camera) {
    const Eigen::Vector3d forward = camera.look_at - camera.position;
    if (!camera.position.allFinite() || !camera.look_at.allFinite() ||
        forward.norm() < 1e-9 || forward.normalized().cross(camera.up).norm() < 1e-6)
        throw std::invalid_argument("Camera '" + camera.name +
                                    "' needs a look_at away from its position and not along up.");
    if (camera.width <= 0 || camera.height <= 0 ||
        !(camera.horizontal_fov_deg > 0 && camera.horizontal_fov_deg < 180) ||
        !(camera.vertical_fov_deg > 0 && camera.vertical_fov_deg < 180) ||
        camera.min_range < 0 || !(camera.max_range > camera.min_range) ||
        camera.noise_base < 0 || camera.noise_per_m2 < 0 ||
        camera.dropout < 0 || camera.dropout >= 1)
        throw std::invalid_argument("Camera '" + camera.name + "' has invalid settings.");
}

}  // namespace

struct SimulatedDepthSensor::Impl {
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data{nullptr, mj_deleteData};
    std::vector<DepthCameraSettings> cameras;
    std::mt19937 rng;

    void captureCamera(const DepthCameraSettings& camera, PointCloud& cloud) {
        // Camera frame: forward along the view, right and up across the image.
        const Eigen::Vector3d forward = (camera.look_at - camera.position).normalized();
        const Eigen::Vector3d right = forward.cross(camera.up).normalized();
        const Eigen::Vector3d up = right.cross(forward);
        const double tan_x = std::tan(camera.horizontal_fov_deg * kPi / 360.0);
        const double tan_y = std::tan(camera.vertical_fov_deg * kPi / 360.0);

        const int rays = camera.width * camera.height;
        std::vector<mjtNum> directions(3 * static_cast<size_t>(rays));
        for (int v = 0; v < camera.height; ++v) {
            for (int u = 0; u < camera.width; ++u) {
                const double x = tan_x * (2.0 * (u + 0.5) / camera.width - 1.0);
                const double y = tan_y * (1.0 - 2.0 * (v + 0.5) / camera.height);
                const Eigen::Vector3d direction = (forward + x * right + y * up).normalized();
                std::copy(direction.data(), direction.data() + 3,
                          directions.data() + 3 * (static_cast<size_t>(v) * camera.width + u));
            }
        }
        std::vector<int> geom(rays);
        std::vector<mjtNum> distance(rays);
        mj_multiRay(model.get(), data.get(), camera.position.data(), directions.data(),
                    nullptr, 1, -1, geom.data(), distance.data(), nullptr, rays,
                    camera.max_range);

        std::uniform_real_distribution<double> uniform(0.0, 1.0);
        std::normal_distribution<double> gaussian(0.0, 1.0);
        for (int ray = 0; ray < rays; ++ray) {
            const double hit = distance[ray];
            if (geom[ray] < 0 || hit < camera.min_range || hit > camera.max_range) continue;
            if (uniform(rng) < camera.dropout) continue;
            const double depth = hit + gaussian(rng) * (camera.noise_base + camera.noise_per_m2 * hit * hit);
            const Eigen::Map<const Eigen::Vector3d> direction(directions.data() + 3 * ray);
            cloud.points.push_back(camera.position + depth * direction);
        }
    }
};

SimulatedDepthSensor::SimulatedDepthSensor(const std::string& scene_xml,
                                           std::vector<DepthCameraSettings> cameras,
                                           const SceneEditor& edit_scene,
                                           std::uint32_t seed)
    : impl_(std::make_unique<Impl>()) {
    if (cameras.empty()) throw std::invalid_argument("The simulated sensor needs at least one camera.");
    for (const auto& camera : cameras) validate(camera);
    char error[1000]{};
    impl_->model.reset(mj_loadXML(scene_xml.c_str(), nullptr, error, sizeof(error)));
    if (!impl_->model)
        throw std::runtime_error("Sensor scene load error (" + scene_xml + "): " + error);
    if (edit_scene) edit_scene(impl_->model.get());
    impl_->data.reset(mj_makeData(impl_->model.get()));
    if (!impl_->data) throw std::runtime_error("Could not create sensor scene data.");
    impl_->cameras = std::move(cameras);
    impl_->rng.seed(seed);
}

SimulatedDepthSensor::~SimulatedDepthSensor() = default;

void SimulatedDepthSensor::setSceneQpos(const Eigen::VectorXd& qpos) {
    if (qpos.size() != impl_->model->nq || !qpos.allFinite())
        throw std::invalid_argument("Sensor scene qpos has the wrong size or is not finite.");
    std::copy(qpos.data(), qpos.data() + qpos.size(), impl_->data->qpos);
}

PointCloud SimulatedDepthSensor::capture() {
    // Geom poses for the current qpos; no physics step.
    mj_kinematics(impl_->model.get(), impl_->data.get());
    PointCloud cloud;
    for (const auto& camera : impl_->cameras) impl_->captureCamera(camera, cloud);
    return cloud;
}
