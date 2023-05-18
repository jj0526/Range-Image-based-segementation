#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <iostream>
#include <fstream>
#include <vector>

typedef pcl::PointXYZI PointType;

int main() {
    std::string bin_file = "path/to/your/file.bin";
    std::ifstream file(bin_file, std::ios::in | std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << bin_file << std::endl;
        return -1;
    }

    // Read the binary file into a vector
    std::vector<PointType> points;
    file.seekg(0, std::ios::end);
    size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);
    points.resize(file_size / sizeof(PointType));
    file.read(reinterpret_cast<char*>(&points[0]), file_size);

    // Create a point cloud object and copy the vector data
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud->points = points;

    // Optional: Apply voxel grid filtering to downsample the point cloud
    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.1, 0.1, 0.1);  // Adjust the leaf size as desired
    pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
    voxel_grid.filter(*filtered_cloud);

    // Create a range image
    pcl::RangeImagePlanar range_image;
    float angular_resolution_x = pcl::deg2rad(0.1f);  // Adjust the angular resolution as desired
    float angular_resolution_y = pcl::deg2rad(0.1f);  // Adjust the angular resolution as desired
    range_image.createFromPointCloudWithFixedSize(*filtered_cloud, angular_resolution_x, angular_resolution_y);

    // Visualize the range image
    pcl::visualization::RangeImageVisualizer range_image_visualizer("Range Image");
    range_image_visualizer.showRangeImage(range_image);
    while (!range_image_visualizer.wasStopped()) {
        // Do nothing, just keep the viewer open
    }

    return 0;
}