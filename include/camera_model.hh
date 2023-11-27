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
        CAMERA_PARAM_TYPE param_type; // 相机参数类型
        unsigned int pix_width;       // 像素宽度
        unsigned int pix_height;      // 像素高度
        float fx;                      // 相机内参-fx（INTRINSICS）
        float fy;                      // 相机内参-fy（INTRINSICS）
        float cx;                      // 相机内参-cx（INTRINSICS）
        float cy;                      // 相机内参-cy（INTRINSICS）
        float h_fov_deg;               // 水平视场角角度（FOV）
        float v_fov_deg;               // 垂直视场角角度（FOV）
        float x_of_ob;                    // 相机外参-x_of_ob
        float y_of_ob;                    // 相机外参-y_of_ob
        float z_of_ob;                    // 相机外参-z_of_ob
        float x_oc_of;                    // 相机外参-x_oc_of
        float y_oc_of;                    // 相机外参-y_oc_of
        float z_oc_of;                    // 相机外参-z_oc_of
        float alpha_deg;                   // 相机外参-alpha
        float beta_deg;                    // 相机外参-beta

        bool check() const
        {
            if (pix_width <= 0 || pix_height <= 0)
            {
                return false;
            }

            if (param_type == INTRINSICS)
            {
                if (fx <= 0.0f || fy <= 0.0f || cx <= 0.0f || cy <= 0.0f)
                {
                    return false;
                }
            }
            else if (param_type == FOV)
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