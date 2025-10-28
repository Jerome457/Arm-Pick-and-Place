#include <dgl_ros/util/cloud.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

namespace dgl
{
namespace util
{
namespace cloud
{
void removeTable(PointCloudRGB::Ptr cloud)
{
    if (cloud->points.empty()) return;

    // Find the lowest z value (ground)
    float min_z = cloud->points[0].z;
    for (const auto& pt : cloud->points)
    {
        if (pt.z < min_z)
            min_z = pt.z;
    }

    // Set a small margin above the ground to keep objects
    float table_margin = 0.001; // 1 cm above ground

    PointCloudRGB::Ptr filtered(new PointCloudRGB);
    for (const auto& pt : cloud->points)
    {
        if (pt.z > min_z + table_margin) // keep points above ground + margin
        {
            filtered->points.push_back(pt);
        }
    }

    filtered->width = filtered->points.size();
    filtered->height = 1;
    filtered->is_dense = true;

    *cloud = *filtered;
}


void passThroughFilter(const std::vector<double>& xyz_lower, const std::vector<double>& xyz_upper,
                       PointCloudRGB::Ptr cloud)
{
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);

  pass.setFilterFieldName("x");
  pass.setFilterLimits(xyz_lower.at(0), xyz_upper.at(0));
  pass.filter(*cloud);

  pass.setFilterFieldName("y");
  pass.setFilterLimits(xyz_lower.at(1), xyz_upper.at(1));
  pass.filter(*cloud);

  pass.setFilterFieldName("z");
  pass.setFilterLimits(xyz_lower.at(2), xyz_upper.at(2));
  pass.filter(*cloud);
}
}  // namespace cloud
}  // namespace util
}  // namespace dgl