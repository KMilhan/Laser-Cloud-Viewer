#include "scale_converter.hpp"
#include <simdjson.h>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace fs = std::filesystem;

bool convert_scale_to_kitti(const std::string &json_file,
                            const std::string &output_dir) {
    simdjson::dom::parser parser;
    simdjson::dom::element root;
    auto error = parser.load(json_file).get(root);
    if (error) {
        std::cerr << "Cannot parse " << json_file << ": "
                  << simdjson::error_message(error) << std::endl;
        return false;
    }
    std::string image_path = std::string(root["image"]);
    fs::path image_fs(image_path);
    std::string stem = image_fs.stem().string();

    fs::create_directories(fs::path(output_dir) / "label_2");

    std::ofstream label((fs::path(output_dir) / "label_2" / (stem + ".txt")).string());
    if (!label.is_open()) {
        std::cerr << "Cannot write label file" << std::endl;
        return false;
    }

    auto anns = root["annotations"].get_array();
    for (auto ann : anns) {
        std::string type = std::string(ann["label"]);
        auto bbox = ann["bbox"];
        float left = float(double(bbox["left"]));
        float top = float(double(bbox["top"]));
        float width = float(double(bbox["width"]));
        float height = float(double(bbox["height"]));
        float right = left + width;
        float bottom = top + height;

        auto dim = ann["dimensions"];
        float h = float(double(dim["height"]));
        float w = float(double(dim["width"]));
        float l = float(double(dim["length"]));

        auto loc = ann["location"];
        float x = float(double(loc["x"]));
        float y = float(double(loc["y"]));
        float z = float(double(loc["z"]));

        float ry = float(double(ann["rotation_y"]));

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
