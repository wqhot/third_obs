#include <third_obs.hh>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

using namespace third_obs;

const float PI = 3.1415926f;

ThirdObs *ThirdObs::get_instance()
{
    static ThirdObs third_obs;
    return &third_obs;
}

bool ThirdObs::set_camera_model(const _camera_model &model)
{
    if (!model.check())
    {
        return false;
    }
    model_ = model;
    return true;
}

void ThirdObs::set_target_bounding_box(const std::vector<_point_3d> &target_bounding_box)
{
    target_bounding_box_ = target_bounding_box;
}

bool ThirdObs::convert_once(const _pose camera, const _pose target, std::vector<_point_2d> &points) const
{
    if (!model_.check())
    {
        return false;
    }
    points.clear();

    Eigen::MatrixXf points_target_tb(4, target_bounding_box_.size());
    for (auto &point : target_bounding_box_)
    {
        points_target_tb(0, points_target_tb.cols() - 1) = point.x;
        points_target_tb(1, points_target_tb.cols() - 1) = point.y;
        points_target_tb(2, points_target_tb.cols() - 1) = point.z;
        points_target_tb(3, points_target_tb.cols() - 1) = 1;
    }

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

    Eigen::MatrixXf points_target_ol(4, target_bounding_box_.size());
    do
    {
        Eigen::Matrix4f R4 = Eigen::Matrix4f::Identity();
        R4.block<3, 1>(0, 3) = Eigen::Vector3f(target.position.x - camera.position.x, target.position.y - camera.position.y, target.position.z - camera.position.z);
        points_target_ol = R4 * points_target_tb;
    } while (false);

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

    Eigen::MatrixXf points_target_of(4, target_bounding_box_.size());
    do
    {
        Eigen::Matrix4f R4 = Eigen::Matrix4f::Identity();
        R4.block<3, 1>(0, 3) = -Eigen::Vector3f(model_.x_ob, model_.y_ob, model_.z_ob);
        points_target_of = R4 * points_target_ob;
    } while (false);

    Eigen::MatrixXf points_target_oc(4, target_bounding_box_.size());
    do
    {
        Eigen::Matrix4f R4 = Eigen::Matrix4f::Identity();
        Eigen::Matrix3f Rz, Rx;
        // Rz << std::sin()
        R4.block<3, 1>(0, 3) = Eigen::Vector3f(model_.x_f, model_.y_f, model_.z_f);
        points_target_f = R4 * points_target_of;
    } while (false);

    //
    return true;
}