#include <third_obs.hh>
#include <iostream>
#include <vector>

int main(int argc, char *argv[])
{
    third_obs::_camera_model camera;
    camera.type_ = third_obs::_camera_model::CAMERA_TYPE::PINEHOLE;
    camera.param_type = third_obs::_camera_model::CAMERA_PARAM_TYPE::INTRINSICS;
    camera.pix_width = 1920;
    camera.pix_height = 1080;
    camera.fx = 960.f;
    camera.fy = 960.f;
    camera.cx = 960.f;
    camera.cy = 540.f;
    camera.x_of_ob = 0.f;
    camera.y_of_ob = 0.f;
    camera.z_of_ob = 0.f;
    camera.x_oc_of = 0.f;
    camera.y_oc_of = 0.f;
    camera.z_oc_of = 0.f;
    camera.alpha_deg = 120.f;
    camera.beta_deg = 0.f;

    if (!third_obs::ThirdObs::get_instance()->set_camera_model(camera))
    {
        std::cout << "camera params error" << std::endl;
        return -1;
    }

    third_obs::_pose car_pose(0, 0, 0, 0, 0, 0);
    third_obs::_pose target_pose(0, 20.f, 0, 45.f, 0, 0);

    // [L1,L2,L3]=deal(5,3,2)
    // X_t_tb=[0 -L2 L2 L2 -L2 -L2 L2 L2 -L2;
    //         0 L1 L1 L1 L1 -L1 -L1 -L1 -L1;
    //         0 L3 L3 -L3 -L3 L3 L3 -L3 -L3]/2
    std::vector<third_obs::_point_3d> bounding_box_points{
        {0.f / 2, 0.f / 2, 0.f / 2},
        {-3.f / 2, 5.f / 2, 2.f / 2},
        {3.f / 2, 5.f / 2, 2.f / 2},
        {3.f / 2, 5.f / 2, -2.f / 2},
        {-3.f / 2, 5.f / 2, -2.f / 2},
        {-3.f / 2, -5.f / 2, 2.f / 2},
        {3.f / 2, -5.f / 2, 2.f / 2},
        {3.f / 2, -5.f / 2, -2.f / 2},
        {-3.f / 2, -5.f / 2, -2.f / 2},
    };

    third_obs::ThirdObs::get_instance()->set_target_bounding_box(bounding_box_points);

    std::vector<third_obs::_point_2d> points_pix;
    std::vector<third_obs::_position_on_screen> points_status;
    third_obs::ThirdObs::get_instance()->convert_once(car_pose, target_pose, points_pix, points_status);

    std::cout << "Points:" << std::endl;
    for (size_t i = 0; i < points_pix.size(); i++)
    {
        std::cout << "[" << points_pix[i].x << ", " << points_pix[i].y << "]: " << points_status[i] << std::endl;
    }

    return 0;
}