#include <camera_model.hh>
#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

using namespace softrender_point;

_camera_model::_camera_model(const std::string &yaml_file_path)
{
    YAML::Node config = YAML::LoadFile(yaml_file_path);

    std::string camera_type = config["camera_type"].as<std::string>();
    if (camera_type != "PINEHOLE" && camera_type != "FISHEYE")
    {
        spdlog::error("Invalid camera_type: {}", camera_type);
        return;
    }

    std::string param_type = config["param_type"].as<std::string>();
    if (param_type != "INTRINSICS" && param_type != "FOV")
    {
        spdlog::error("Invalid param_type: {}", param_type);
        return;
    }

    int pix_width = config["pix_width"].as<int>();
    int pix_height = config["pix_height"].as<int>();
    if (pix_width <= 0 || pix_height <= 0)
    {
        spdlog::error("Invalid resolution: {}x{}", pix_width, pix_height);
        return;
    }

    YAML::Node intrinsics = config["camera_intrinscis"];
    float fx = intrinsics["fx"].as<float>();
    float fy = intrinsics["fy"].as<float>();
    float cx = intrinsics["cx"].as<float>();
    float cy = intrinsics["cy"].as<float>();

    YAML::Node extrinsics = config["camera_extrinscis"];
    float x_of_ob = extrinsics["x_of_ob"].as<float>();
    float y_of_ob = extrinsics["y_of_ob"].as<float>();
    float z_of_ob = extrinsics["z_of_ob"].as<float>();
    float x_oc_of = extrinsics["x_oc_of"].as<float>();
    float y_oc_of = extrinsics["y_oc_of"].as<float>();
    float z_oc_of = extrinsics["z_oc_of"].as<float>();
    float alpha_deg = extrinsics["alpha_deg"].as<float>();
    float beta_deg = extrinsics["beta_deg"].as<float>();

    if (camera_type == "PINEHOLE")
    {
        this->type_ = PINEHOLE;
    }
    else if (camera_type == "FISHEYE")
    {
        this->type_ = FISHEYE;
    }

    if (param_type == "INTRINSICS")
    {
        this->param_type = INTRINSICS;
    }
    else if (param_type == "FOV")
    {
        this->param_type = FOV;
    }

    this->fx = fx;
    this->fy = fy;
    this->cx = cx;
    this->cy = cy;
    this->x_of_ob = x_of_ob;
    this->y_of_ob = y_of_ob;
    this->z_of_ob = z_of_ob;
    this->x_oc_of = x_oc_of;
    this->y_oc_of = y_oc_of;
    this->z_oc_of = z_oc_of;
    this->alpha_deg = alpha_deg;
    this->beta_deg = beta_deg;

    if (!check())
    {
        spdlog::error("camera model check error");
        return;
    }
}