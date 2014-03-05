#include "commandline.hpp"
#include <sstream>


int main(int ac, char *av[]) {
    cout << ac << "Arguments received" << endl;

    float voxel_leaf_size;
    int max_iteration;
    int distnace_thresh;
    double cluster_tolerance;
    int min_cluster_size;
    int max_cluster_size;
    if (ac != 7)
        cout << "Invalid arguments" << endl;
    else {

    }

    model cloud_model;
    std::vector <string> file_path;

    for (int i = 0;; i++) {
        string temp = "./temp/" + boost::lexical_cast<std::string>(i) + ".pcd";
        boost::filesystem::path p(temp);
        if (boost::filesystem::exists(p)) {
            file_path.push_back(temp);
            cout << file_path.back() << endl; // find until last MK_i file then break.
        } else
            break;
    }
    cout << "Opening..." << endl;
    cout << cloud_model.open_files(file_path) << "Files" << endl;
    cout << "In segmentation..." << endl;


    if (ac == 7) {
        cout << cloud_model.segmentation();
    } else
        cout << cloud_model.segmentation();

//UPDATE TEMP DIR
    for (int i = 0;; i++) {
        string temp = "./temp/" + boost::lexical_cast<std::string>(i) + ".pcd";
        boost::filesystem::path p(temp);
        if (boost::filesystem::exists(p)) {
            boost::filesystem::remove_all(p); // delete until last MK_i file then break.
            cout << "Deleting... " << temp << endl;
        } else
            break;

    }
    cout << "Saving..." << cloud_model.save_files("./temp/");

    cloud_model.visualize_cloud();

    return 0;
}
