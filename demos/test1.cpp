#include <softrender_point.hh>
#include <iostream>
#include <vector>

int main(int argc, char *argv[])
{
    // 建立相机模型 输入参数：相机内参yaml文件路径
    softrender_point::_camera_model camera(argv[1]);

    // 应用相机模型（可多次重复设置相机参数）
    if (!softrender_point::_softrendr_point::get_instance()->set_camera_model(camera))
    {
        std::cout << "camera params error" << std::endl;
        return -1;
    }

    // 观察者坐标系在大地坐标系下的位姿 （x(m), y(m), z(m), yaw(deg), pitch(deg), roll(deg)）
    softrender_point::_pose car_pose(0, 0, 0, 0, 0, 0);
    // 目标坐标系在大地坐标系下的位姿 （x(m), y(m), z(m), yaw(deg), pitch(deg), roll(deg)）
    softrender_point::_pose target_pose(0, 20.f, 0, 45.f, 0, 0);

    // [L1,L2,L3]=deal(5,3,2)
    // X_t_tb=[0 -L2 L2 L2 -L2 -L2 L2 L2 -L2;
    //         0 L1 L1 L1 L1 -L1 -L1 -L1 -L1;
    //         0 L3 L3 -L3 -L3 L3 L3 -L3 -L3]/2
    // 目标轮廓点在目标坐标系的坐标 （x(m), y(m), z(m)）
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

    // 应用轮廓点（可多次重复设置轮廓点）
    softrender_point::_softrendr_point::get_instance()->set_target_points(bounding_box_points);

    std::vector<softrender_point::_point_2d> points_pix; // 映射后轮廓点的像素坐标 画面左上角为零点
    std::vector<softrender_point::_position_on_screen> points_status; // 映射后轮廓点和屏幕的关系（在屏幕中/不在屏幕中（极坐标角度值））
    softrender_point::_softrendr_point::get_instance()->convert_once(car_pose, target_pose, points_pix, points_status); // 计算一次映射

    std::cout << "Points:" << std::endl;
    for (size_t i = 0; i < points_pix.size(); i++)
    {
        std::cout << "[" << points_pix[i].x << ", " << points_pix[i].y << "]: " << points_status[i].is_center << std::endl;
    }

    return 0;
}