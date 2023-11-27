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
    };

    class _point_2d
    {
    public:
        float x;
        float y;
    };

    class _pose
    {
    public:
        _point_3d position; // 墨卡托坐标系
        float yaw_deg;      // 旋转顺序: ZYX
        float pitch_deg;
        float roll_deg;
    };

    class ThirdObs
    {
    public:
        ThirdObs *get_instance();
        bool convert_once(const _pose camera, const _pose target, std::vector<_point_2d> &points) const;
        bool set_camera_model(const _camera_model &model);
        void set_target_bounding_box(const std::vector<_point_3d> &target_bounding_box);

    private:
        _camera_model model_;
        std::vector<_point_3d> target_bounding_box_;
    };
};

#endif