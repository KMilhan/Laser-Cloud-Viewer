#include "kitti/scale_converter.hpp"
#include <iostream>

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "Usage: scale_to_kitti_cli <scale_json> <output_dir>" << std::endl;
        return 1;
    }
    if (!convert_scale_to_kitti(argv[1], argv[2])) {
        std::cerr << "Conversion failed" << std::endl;
        return 1;
    }
    std::cout << "Conversion successful" << std::endl;
    return 0;
}
