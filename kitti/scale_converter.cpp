#include "scale_converter.hpp"
#include <json/json.h>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace fs = std::filesystem;

bool convert_scale_to_kitti(const std::string &json_file,
                            const std::string &output_dir) {
    std::ifstream in(json_file);
    if (!in.is_open()) {
        std::cerr << "Cannot open " << json_file << std::endl;
        return false;
    }
    Json::Value root;
    in >> root;
    if (!root.isObject()) {
        std::cerr << "Invalid JSON format" << std::endl;
        return false;
    }
    std::string image_path = root.get("image", "").asString();
    fs::path image_fs(image_path);
    std::string stem = image_fs.stem().string();

    fs::create_directories(fs::path(output_dir) / "label_2");

    std::ofstream label((fs::path(output_dir) / "label_2" / (stem + ".txt")).string());
    if (!label.is_open()) {
        std::cerr << "Cannot write label file" << std::endl;
        return false;
    }

    const Json::Value anns = root["annotations"];
    for (const auto &ann : anns) {
        std::string type = ann.get("label", "").asString();
        const auto &bbox = ann["bbox"];
        float left = bbox.get("left", 0.f).asFloat();
        float top = bbox.get("top", 0.f).asFloat();
        float width = bbox.get("width", 0.f).asFloat();
        float height = bbox.get("height", 0.f).asFloat();
        float right = left + width;
        float bottom = top + height;

        const auto &dim = ann["dimensions"];
        float h = dim.get("height", 0.f).asFloat();
        float w = dim.get("width", 0.f).asFloat();
        float l = dim.get("length", 0.f).asFloat();

        const auto &loc = ann["location"];
        float x = loc.get("x", 0.f).asFloat();
        float y = loc.get("y", 0.f).asFloat();
        float z = loc.get("z", 0.f).asFloat();

        float ry = ann.get("rotation_y", 0.f).asFloat();

        label << type << " 0 0 0 "
              << left << ' ' << top << ' ' << right << ' ' << bottom << ' '
              << h << ' ' << w << ' ' << l << ' '
              << x << ' ' << y << ' ' << z << ' '
              << ry << '\n';
    }

    label.close();

    // copy image if exists
    if (!image_path.empty()) {
        fs::create_directories(fs::path(output_dir) / "image_2");
        try {
            fs::copy_file(image_fs, fs::path(output_dir) / "image_2" / image_fs.filename(),
                           fs::copy_options::overwrite_existing);
        } catch (const std::exception &e) {
            std::cerr << "Failed to copy image: " << e.what() << std::endl;
        }
    }

    return true;
}
