
#include <random>
#include <chrono>

// #include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
// #include <pcl/common/distances.h>
#include <pcl/point_types.h>

#include <pcl/octree/octree.h>

#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.h>

const int TEST_NUM = 20;
const int max_answers = 500;

void pcl_octree_radiusSearch(std::vector<pcl::PointXYZ> &points,
                             std::vector<pcl::PointXYZ> &queries,
                             std::vector<float> &radiuses)
{

    // host buffers
    std::vector<int> indices;
    pcl::Indices indices_host;
    std::vector<float> pointRadiusSquaredDistance;

    // reserve
    indices.reserve(points.size());
    indices_host.reserve(points.size());
    pointRadiusSquaredDistance.reserve(points.size());

    // prepare host cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_host(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_host->width = points.size();
    cloud_host->height = 1;
    cloud_host->resize(cloud_host->width * cloud_host->height);

    for (std::size_t i = 0; i < cloud_host->size(); ++i)
    {
        (*cloud_host)[i].x = points[i].x;
        (*cloud_host)[i].y = points[i].y;
        (*cloud_host)[i].z = points[i].z;
    }

    float host_octree_resolution = 25.f;
    std::cout << "[!] Host octree resolution: " << host_octree_resolution << std::endl
              << std::endl;

    // build host octree
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_host(host_octree_resolution);
    octree_host.setInputCloud(cloud_host);
    octree_host.addPointsFromInputCloud();

    for (int idx = 0; idx < TEST_NUM; idx++)
    {

        auto start = std::chrono::steady_clock::now();

        for (std::size_t i = 0; i < queries.size(); ++i)
            octree_host.radiusSearch(queries[i], radiuses[i], indices_host, pointRadiusSquaredDistance, max_answers);

        auto stop = std::chrono::steady_clock::now();
        auto ipp_time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count() / 1000.0;

        printf("pcl radius search took %.3f milliseconds \n", ipp_time);
    }
}

void cuda_octree_radiusSearch(std::vector<pcl::PointXYZ> &points,
                              std::vector<pcl::PointXYZ> &queries,
                              std::vector<float> &radiuses)
{
    // prepare device cloud
    pcl::gpu::Octree::PointCloud cloud_device;
    cloud_device.upload(points);

    // prepare queries_device
    pcl::gpu::Octree::Queries queries_device;
    pcl::gpu::Octree::Radiuses radiuses_device;
    queries_device.upload(queries);
    radiuses_device.upload(radiuses);

    // build device octree
    pcl::gpu::Octree octree_device;
    octree_device.setCloud(cloud_device);
    octree_device.build();
    // octree_device.internalDownload();

    pcl::gpu::NeighborIndices result_device(queries_device.size(), max_answers);

    for (int i = 0; i < TEST_NUM; i++)
    {

        auto start = std::chrono::steady_clock::now();

        octree_device.radiusSearch(queries_device, radiuses_device, max_answers, result_device);

        auto stop = std::chrono::steady_clock::now();
        auto ipp_time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count() / 1000.0;

        printf("cuda radius search took %.3f milliseconds \n", ipp_time);
    }

    std::vector<int> downloaded;
    // std::vector<float> dists_device_downloaded;
    result_device.data.download(downloaded);
    // sqr_distance.download(dists_device_downloaded);

    printf("result size is %ld \n\n\n", downloaded.size());
}

void pcl_octree_approxNearestSearch(std::vector<pcl::PointXYZ> &points, std::vector<pcl::PointXYZ> &queries)
{

    // prepare host cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_host(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_host->width = points.size();
    cloud_host->height = 1;
    cloud_host->resize(cloud_host->width * cloud_host->height);

    for (std::size_t i = 0; i < cloud_host->size(); ++i)
    {
        (*cloud_host)[i].x = points[i].x;
        (*cloud_host)[i].y = points[i].y;
        (*cloud_host)[i].z = points[i].z;
    }

    float host_octree_resolution = 25.f;
    std::cout << "[!] Host octree resolution: " << host_octree_resolution << std::endl
              << std::endl;

    // build host octree
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_host(host_octree_resolution);
    octree_host.setInputCloud(cloud_host);
    octree_host.addPointsFromInputCloud();

    pcl::index_t inds;
    float dist;

    for (int idx = 0; idx < TEST_NUM; idx++)
    {

        auto start = std::chrono::steady_clock::now();

        for (std::size_t i = 0; i < queries.size(); ++i)
            octree_host.approxNearestSearch(queries[i], inds, dist);

        auto stop = std::chrono::steady_clock::now();
        auto ipp_time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count() / 1000.0;

        printf("pcl approx nearest took %.3f milliseconds \n", ipp_time);
    }
}

void cuda_octree_approxNearestSearch(std::vector<pcl::PointXYZ> &points, std::vector<pcl::PointXYZ> &queries)
{

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

    // octree_device.internalDownload();

    pcl::gpu::Octree::ResultSqrDists sqr_distance;

    for (int i = 0; i < TEST_NUM; i++)
    {

        auto start = std::chrono::steady_clock::now();

        octree_device.approxNearestSearch(queries_device, result_device, sqr_distance);

        auto stop = std::chrono::steady_clock::now();
        auto ipp_time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count() / 1000.0;

        printf("cuda approx nearest took %.3f milliseconds \n", ipp_time);
    }

    std::vector<int> downloaded;
    std::vector<float> dists_device_downloaded;
    result_device.data.download(downloaded);
    sqr_distance.download(dists_device_downloaded);

    printf("result size is %ld \n\n\n", downloaded.size());
}

int main(int argc, char **argv)
{

    //==========================================================================================

    constexpr int max_answers = 500;
    std::size_t data_size = 871000;
    std::size_t query_size = 10000;
    float max_radius = 1024.f / 15.f;

    std::vector<pcl::PointXYZ> points;
    std::vector<pcl::PointXYZ> queries;
    std::vector<float> radiuses;
    std::vector<int> indices;

    // random point value vary from 0 to 1024

    std::mt19937 gen(10);

    std::uniform_real_distribution<> dis(0.0, 1024.0);

    std::uniform_real_distribution<> disRadius(0.0, max_radius);

    points.resize(data_size);

    for (std::size_t i = 0; i < data_size; ++i)
    {
        points[i].x = dis(gen);
        points[i].y = dis(gen);
        points[i].z = dis(gen);
    }

    queries.resize(query_size);
    radiuses.resize(query_size);

    for (std::size_t i = 0; i < query_size; ++i)
    {
        queries[i].x = dis(gen);
        queries[i].y = dis(gen);
        queries[i].z = dis(gen);
        radiuses[i] = disRadius(gen);
    };

    // for (int i = 0; i < query_size; i++)
    // {
    //     printf("%.4f \n", radiuses[i]);
    // }

    // cuda device cloud

    cuda_octree_approxNearestSearch(points, queries);
    pcl_octree_approxNearestSearch(points, queries);

    cuda_octree_radiusSearch(points, queries, radiuses);
    pcl_octree_radiusSearch(points, queries, radiuses);
}