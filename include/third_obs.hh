#ifndef _THIRD_OBS_HH
#define _THIRD_OBS_HH
#include "camera_model.hh"
#include <vector>

namespace third_obs
{
    class _point_3d
    {
    public:
        float x;
        float y;
        float z;
        _point_3d() : x(0), y(0), z(0) {}
        _point_3d(float x, float y, float z) : x(x), y(y), z(z) {}
    };

    class _point_2d
    {
    public:
        float x;
        float y;
        _point_2d() : x(0), y(0) {}
        _point_2d(float x, float y) : x(x), y(y) {}
    };

    class _pose
    {
    public:
        _point_3d position; // 墨卡托坐标系
        float yaw_deg;      // 旋转顺序: ZYX
        float pitch_deg;
        float roll_deg;
        _pose() : yaw_deg(0), pitch_deg(0), roll_deg(0) {}
        _pose(float x, float y, float z, float yaw_deg, float pitch_deg, float roll_deg) : position(x, y, z), yaw_deg(yaw_deg), pitch_deg(pitch_deg), roll_deg(roll_deg) {}
    };

    enum _position_on_screen
    {
        error = -1,
        center = 0,
        top_left,
        top_right,
        bottom_left,
        bottom_right,
    };

    class ThirdObs
    {
    public:
        static ThirdObs *get_instance();
        bool convert_once(const _pose camera, const _pose target, std::vector<_point_2d> &points, std::vector<_position_on_screen> &point_status) const;
        bool set_camera_model(const _camera_model &model);
        void set_target_bounding_box(const std::vector<_point_3d> &target_bounding_box);

    private:
        _camera_model model_;
        std::vector<_point_3d> target_bounding_box_;
    };
};

#endif