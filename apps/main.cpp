# include <iostream>
# include <numeric>
# include <cxxopts.hpp>

# include "ring_buffer.hpp"
# include "unique_pointer.hpp"

namespace fs = std::filesystem;

int main(int argc, char *argv[]) {
    cxxopts::Options options("marker-pose", "options to configure marker pose estimation pipeline");

    options.add_options()("image", "Path to the image to process", cxxopts::value<fs::path>());
    options.add_options()("marker-side", "Length of the marker's side in meters", cxxopts::value<float>()->default_value("10.0"));

    auto args = options.parse(argc, argv);

    return 0;
}
