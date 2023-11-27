#ifndef _CMAERA_MODEL_HH
#define _CMAERA_MODEL_HH

namespace third_obs
{
    class _camera_model
    {
    public:
        enum CAMERA_TYPE
        {
            PINEHOLE = 0, // 针孔
            FISHEYE       // 鱼眼
        };

        enum CAMERA_PARAM_TYPE
        {
            INTRINSICS = 0, // 使用相机内参描述相机参数
            FOV             // 使用视场角描述相机参数
        };

        CAMERA_TYPE type_;             // 相机类型
        CAMERA_PARAM_TYPE param_type_; // 相机参数类型
        unsigned int pix_width_;       // 像素宽度
        unsigned int pix_height_;      // 像素高度
        float fx;                      // 相机内参-fx（INTRINSICS）
        float fy;                      // 相机内参-fy（INTRINSICS）
        float cx;                      // 相机内参-cx（INTRINSICS）
        float cy;                      // 相机内参-cy（INTRINSICS）
        float h_fov_deg;               // 水平视场角角度（FOV）
        float v_fov_deg;               // 垂直视场角角度（FOV）
        float x_ob;                    // 相机外参-x_ob
        float y_ob;                    // 相机外参-y_ob
        float z_ob;                    // 相机外参-z_ob
        float x_of;                    // 相机外参-x_of
        float y_of;                    // 相机外参-y_of
        float z_of;                    // 相机外参-z_of

        bool check() const
        {
            if (pix_width_ <= 0 || pix_height_ <= 0)
            {
                return false;
            }

            if (param_type_ == INTRINSICS)
            {
                if (fx <= 0.0f || fy <= 0.0f || cx <= 0.0f || cy <= 0.0f)
                {
                    return false;
                }
            }
            else if (param_type_ == FOV)
            {
                if ((h_fov_deg > 0.0f) && (v_fov_deg > 0.0f))
                {
                    return true;
                }
            }
            return true;
        }
    };
};

#endif