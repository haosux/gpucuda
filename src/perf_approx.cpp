
#include <random>
#include <chrono>

// #include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
// #include <pcl/common/distances.h>
#include <pcl/point_types.h>

#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.h>

int main(int argc, char **argv)
{

    std::mt19937 gen(10);

    // random point value vary from 0 to 1024
    std::uniform_real_distribution<> dis(0.0, 1024.0);

    // create a point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Generate pointcloud data
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (int i = 0; i < 10; i++)
    {
        printf("%.4f \n", dis(gen));
    }

    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        (*cloud)[i].x = dis(gen);
        (*cloud)[i].y = dis(gen);
        (*cloud)[i].z = dis(gen);
    }

    //==========================================================================================

    constexpr int max_answers = 500;
    std::size_t data_size = 8710000;
    std::size_t query_size = 10000;
    std::vector<pcl::PointXYZ> points;
    std::vector<pcl::PointXYZ> queries;
    std::vector<float> radiuses;
    std::vector<int> indices;

    points.resize(data_size);

    for (std::size_t i = 0; i < data_size; ++i)
    {
        points[i].x = dis(gen);
        points[i].y = dis(gen);
        points[i].z = dis(gen);
    }

    queries.resize(query_size);

    for (std::size_t i = 0; i < query_size; ++i)
    {
        queries[i].x = dis(gen);
        queries[i].y = dis(gen);
        queries[i].z = dis(gen);
        // radiuses[i]  = ((float)rand())/(float)RAND_MAX * max_radius;
    };

    // cuda device cloud

    // prepare device cloud
    pcl::gpu::Octree::PointCloud cloud_device;
    cloud_device.upload(points);

    // prepare queries_device
    pcl::gpu::Octree::Queries queries_device;
    // pcl::gpu::Octree::Radiuses radiuses_device;
    queries_device.upload(queries);

    pcl::gpu::NeighborIndices result_device(queries_device.size(), max_answers);

    // build device octree
    pcl::gpu::Octree octree_device;
    octree_device.setCloud(cloud_device);

    octree_device.build();

    octree_device.internalDownload();

    pcl::gpu::Octree::ResultSqrDists sqr_distance;

    for (int i = 0; i < 100; i++)
    {

        auto start = std::chrono::steady_clock::now();

        octree_device.approxNearestSearch(queries_device, result_device, sqr_distance);

        auto stop = std::chrono::steady_clock::now();
        auto ipp_time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count() / 1000.0;

        printf("approx cuda took %.3f milliseconds \n", ipp_time);
    }

    std::vector<int> downloaded;
    std::vector<float> dists_device_downloaded;
    result_device.data.download(downloaded);
    sqr_distance.download(dists_device_downloaded);

    printf("result size is %ld \n\n\n", downloaded.size());
}