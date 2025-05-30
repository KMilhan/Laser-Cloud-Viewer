#ifndef SCALE_CONVERTER_HPP
#define SCALE_CONVERTER_HPP

#include <string>

bool convert_scale_to_kitti(const std::string &json_file,
                            const std::string &output_dir);

#endif // SCALE_CONVERTER_HPP
