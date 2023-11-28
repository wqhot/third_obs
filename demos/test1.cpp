#include <third_obs.hh>
#include <iostream>
#include <vector>

int main(int argc, char *argv[])
{
    softrender_point::_camera_model camera(argv[1]);

    if (!softrender_point::ThirdObs::get_instance()->set_camera_model(camera))
    {
        std::cout << "camera params error" << std::endl;
        return -1;
    }

    softrender_point::_pose car_pose(0, 0, 0, 0, 0, 0);
    softrender_point::_pose target_pose(0, 20.f, 0, 45.f, 0, 0);

    // [L1,L2,L3]=deal(5,3,2)
    // X_t_tb=[0 -L2 L2 L2 -L2 -L2 L2 L2 -L2;
    //         0 L1 L1 L1 L1 -L1 -L1 -L1 -L1;
    //         0 L3 L3 -L3 -L3 L3 L3 -L3 -L3]/2
    std::vector<softrender_point::_point_3d> bounding_box_points{
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

    softrender_point::ThirdObs::get_instance()->set_target_bounding_box(bounding_box_points);

    std::vector<softrender_point::_point_2d> points_pix;
    std::vector<softrender_point::_position_on_screen> points_status;
    softrender_point::ThirdObs::get_instance()->convert_once(car_pose, target_pose, points_pix, points_status);

    std::cout << "Points:" << std::endl;
    for (size_t i = 0; i < points_pix.size(); i++)
    {
        std::cout << "[" << points_pix[i].x << ", " << points_pix[i].y << "]: " << points_status[i].is_center << std::endl;
    }

    return 0;
}