#include <third_obs.hh>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/cfg/env.h>
#include <spdlog/fmt/ostr.h>
#include <cmath>
#include <chrono>

using namespace third_obs;

const float PI = 3.1415926f;

ThirdObs::ThirdObs()
{
    spdlog::cfg::load_env_levels();
    auto console = spdlog::stdout_color_mt("THIRDOBS");
    spdlog::set_default_logger(console);

    spdlog::flush_every(std::chrono::seconds(1));
}

ThirdObs *ThirdObs::get_instance()
{
    static ThirdObs third_obs;
    return &third_obs;
}

bool ThirdObs::set_camera_model(const _camera_model &model)
{
    if (!model.check())
    {
        spdlog::warn("camera model error");
        return false;
    }
    model_ = model;
    return true;
}

void ThirdObs::set_target_bounding_box(const std::vector<_point_3d> &target_bounding_box)
{
    target_bounding_box_ = target_bounding_box;
    for (auto &p : target_bounding_box_)
    {
        spdlog::debug("target bounding box: [{}, {}, {}]", p.x, p.y, p.z);
    }
}

bool ThirdObs::convert_once(const _pose camera, const _pose target, std::vector<_point_2d> &points, std::vector<_position_on_screen> &point_status) const
{
    if (!model_.check())
    {
        spdlog::warn("camera model error");
        return false;
    }
    points.resize(target_bounding_box_.size());
    point_status.resize(target_bounding_box_.size(), _position_on_screen::error);

    Eigen::MatrixXf points_target_tb(4, target_bounding_box_.size());
    for (std::size_t i = 0; i < target_bounding_box_.size(); i++)
    {
        const _point_3d &point = target_bounding_box_[i];
        points_target_tb(0, i) = point.x;
        points_target_tb(1, i) = point.y;
        points_target_tb(2, i) = point.z;
        points_target_tb(3, i) = 1;
    }
    spdlog::debug("points_target_tb inited: \n{}", points_target_tb);

    do
    {
        Eigen::AngleAxisf angle_axis_yaw(target.yaw_deg / 180.0f * PI, Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf angle_axis_pitch(target.pitch_deg / 180.0f * PI, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf angle_axis_roll(target.roll_deg / 180.0f * PI, Eigen::Vector3f::UnitX());
        Eigen::Quaternionf q = angle_axis_yaw * angle_axis_pitch * angle_axis_roll;
        Eigen::Matrix3f R = q.toRotationMatrix();
        Eigen::Matrix4f R4 = Eigen::Matrix4f::Identity();
        R4.block<3, 3>(0, 0) = R;
        points_target_tb = R4 * points_target_tb;
    } while (false);
    spdlog::debug("points_target_tb: \n{}", points_target_tb);

    Eigen::MatrixXf points_target_ol(4, target_bounding_box_.size());
    do
    {
        Eigen::Matrix4f R4 = Eigen::Matrix4f::Identity();
        R4.block<3, 1>(0, 3) = Eigen::Vector3f(target.position.x - camera.position.x, target.position.y - camera.position.y, target.position.z - camera.position.z);
        points_target_ol = R4 * points_target_tb;
    } while (false);
    spdlog::debug("points_target_ol: \n{}", points_target_ol);

    Eigen::MatrixXf points_target_ob(4, target_bounding_box_.size());
    do
    {
        Eigen::AngleAxisf angle_axis_yaw(camera.yaw_deg / 180.0f * PI, Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf angle_axis_pitch(camera.pitch_deg / 180.0f * PI, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf angle_axis_roll(camera.roll_deg / 180.0f * PI, Eigen::Vector3f::UnitX());
        Eigen::Quaternionf q = angle_axis_yaw * angle_axis_pitch * angle_axis_roll;
        Eigen::Matrix3f R = q.toRotationMatrix();
        Eigen::Matrix4f R4 = Eigen::Matrix4f::Identity();
        R4.block<3, 3>(0, 0) = R;
        points_target_ob = R4 * points_target_ol;
    } while (false);
    spdlog::debug("points_target_ob: \n{}", points_target_ob);

    Eigen::MatrixXf points_target_of(4, target_bounding_box_.size());
    do
    {
        Eigen::Matrix4f R4 = Eigen::Matrix4f::Identity();
        R4.block<3, 1>(0, 3) = -Eigen::Vector3f(model_.x_of_ob, model_.y_of_ob, model_.z_of_ob);
        points_target_of = R4 * points_target_ob;
    } while (false);
    spdlog::debug("points_target_of: \n{}", points_target_of);

    Eigen::MatrixXf points_target_oc(4, target_bounding_box_.size());
    do
    {
        Eigen::Matrix4f R4 = Eigen::Matrix4f::Identity();
        Eigen::Matrix3f Rz, Rx;
        Rz << std::sin(model_.alpha_deg / 180.0f * PI), -std::cos(model_.alpha_deg / 180.0f * PI), 0,
            std::cos(model_.alpha_deg / 180.0f * PI), std::sin(model_.alpha_deg / 180.0f * PI), 0,
            0, 0, 1;
        Rx << 1, 0, 0,
            0, std::sin(model_.beta_deg / 180.0f * PI), -std::cos(model_.beta_deg / 180.0f * PI),
            0, std::cos(model_.beta_deg / 180.0f * PI), std::sin(model_.beta_deg / 180.0f * PI);
        Eigen::Matrix3f R = Rx * Rz;
        R4.block<3, 3>(0, 0) = R;
        R4.block<3, 1>(0, 3) = -R * Eigen::Vector3f(model_.x_oc_of, model_.y_oc_of, model_.z_oc_of);
        points_target_oc = R4 * points_target_of;
    } while (false);
    spdlog::debug("points_target_oc: \n{}", points_target_oc);

    Eigen::MatrixXf points_target_c(4, target_bounding_box_.size());
    do
    {
        Eigen::Matrix4f R4 = Eigen::Matrix4f::Identity();
        R4.block<3, 1>(0, 3) = Eigen::Vector3f(model_.x_oc_of, model_.y_oc_of, model_.z_oc_of);
        points_target_c = R4 * points_target_of;
    } while (false);
    spdlog::debug("points_target_c: \n{}", points_target_c);

    std::vector<float> alphas_c(target_bounding_box_.size(), 0.0f);
    std::vector<float> betas_c(target_bounding_box_.size(), 0.0f);
    for (std::size_t i = 0; i < target_bounding_box_.size(); i++)
    {
        if (points_target_c(0, i) > 0 && points_target_c(1, i) >= 0)
        {
            alphas_c[i] = (std::atan(points_target_c(1, i) / points_target_c(0, i)) * 180.0f / PI);
        }
        else if (points_target_c(0, i) <= 0 && points_target_c(1, i) >= 0)
        {
            alphas_c[i] = (std::atan(std::abs(points_target_c(0, i) / points_target_c(1, i))) * 180.0f / PI + 90.0f);
        }
        else if (points_target_c(0, i) < 0 && points_target_c(1, i) <= 0)
        {
            alphas_c[i] = (std::atan(std::abs(points_target_c(1, i) / points_target_c(0, i))) * 180.0f / PI + 180.0f);
        }
        else if (points_target_c(0, i) >= 0 && points_target_c(1, i) < 0)
        {
            alphas_c[i] = (std::atan(std::abs(points_target_c(0, i) / points_target_c(1, i))) * 180.0f / PI + 270.0f);
        }
        else
        {
            alphas_c[i] = (-180.0f);
        }

        if (points_target_c(0, i) != 0 || points_target_c(1, i) != 0)
        {
            betas_c[i] = std::atan(points_target_c(2, i) / std::sqrt(points_target_c(0, i) * points_target_c(0, i) + points_target_c(1, i) * points_target_c(1, i))) * 180.0f / PI;
        }
        else if (points_target_c(0, i) == 0 && points_target_c(1, i) == 0 && points_target_c(2, i) > 0)
        {
            betas_c[i] = 90.0f;
        }
        else if (points_target_c(0, i) == 0 && points_target_c(1, i) == 0 && points_target_c(2, i) < 0)
        {
            betas_c[i] = -90.0f;
        }
        else
        {
            betas_c[i] = -180.0f;
        }

        int up_down = 0;
        int left_right = 0;
        if (points_target_oc(2, i) <= -model_.fx / model_.cx * points_target_oc(0, i) && points_target_oc(0, i) < 0)
        {
            left_right = -1;
        }
        else if (points_target_oc(2, i) <= model_.fx / (model_.pix_width - model_.cx) * points_target_oc(0, i) && points_target_oc(0, i) > 0)
        {
            left_right = 1;
        }
        else if (points_target_oc(2, i) > model_.fx / (model_.pix_width - model_.cx) * points_target_oc(0, i) && points_target_oc(2, i) > -model_.fx / model_.cx * points_target_oc(0, i))
        {
            left_right = 0;
        }
        else
        {
            left_right = -2;
        }

        if (points_target_oc(2, i) <= -model_.fy / model_.cy * points_target_oc(1, i) && points_target_oc(1, i) < 0)
        {
            up_down = -1;
        }
        else if (points_target_oc(2, i) <= model_.fy / (model_.pix_height - model_.cy) * points_target_oc(1, i) && points_target_oc(1, i) > 0)
        {
            up_down = 1;
        }
        else if (points_target_oc(2, i) > model_.fy / (model_.pix_height - model_.cy) * points_target_oc(1, i) && points_target_oc(2, i) > -model_.fy / model_.cy * points_target_oc(1, i))
        {
            up_down = 0;
        }
        else
        {
            up_down = -2;
        }

        if (up_down == 0 && left_right == 0)
        {
            point_status[i] = _position_on_screen::center;
        }

        spdlog::debug("[{}] Alpha: {}, Beta: {}", i, alphas_c[i], betas_c[i]);
    }

    Eigen::MatrixXf points_target_pix(4, target_bounding_box_.size());
    do
    {
        Eigen::Matrix4f R4 = Eigen::Matrix4f::Zero();
        R4(0, 0) = model_.fx;
        R4(0, 2) = model_.cx;
        R4(1, 1) = model_.fy;
        R4(1, 2) = model_.cy;
        R4(2, 2) = 1;
        spdlog::debug("camera intrinsics: \n{}", R4);
        Eigen::Matrix<float, 4, Eigen::Dynamic> points_target_pix_z = points_target_oc.row(2).replicate(points_target_oc.rows(), 1);
        spdlog::debug("points_target_pix_z: \n{}", points_target_pix_z);
        points_target_pix = R4 * (points_target_oc.cwiseQuotient(points_target_pix_z));
    } while (false);
    spdlog::debug("points_target_pix: \n{}", points_target_pix);

    for (std::size_t i = 0; i < target_bounding_box_.size(); i++)
    {
        points[i].x = points_target_pix(0, i);
        points[i].y = points_target_pix(1, i);
    }

    return true;
}