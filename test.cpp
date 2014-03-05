#include "typedefs.h"
#include "file/opener.hpp"
#include "simple_vis/simple_vis.hpp"
#include "model.hpp"


int main(int argc, char **argv) {
    std::cout << "Initilized" << std::endl;
    model cloud_model;
    std::vector <std::string> file_path;
    file_path.push_back("./data/P05S10.txt");
    file_path.push_back("./data/P05S11.txt");
    file_path.push_back("./data/P05S13.txt.pcd");

    std::cout << cloud_model.open_files(file_path) << std::endl;
    std::cout << cloud_model.registration() << std::endl;
    std::cout << cloud_model.segmentation() << std::endl;
    cloud_model.visualize_cloud();
    return 0;
}

