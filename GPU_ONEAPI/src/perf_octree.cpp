

#include <random>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/octree/octree.h>

#include <pcl/oneapi/octree/octree.hpp>
#include <pcl/oneapi/containers/device_array.h>

#include <dpct/dpct.hpp>
#include <iomanip>


const int TEST_NUM = 20;
const int max_answers = 500;

const std::size_t data_size = 871000;
const std::size_t query_size = 10000;
const float max_radius = 1024.f / 15.f;

void oneapi_octree_approxNearestSearch(std::vector<pcl::PointXYZ> &points, std::vector<pcl::PointXYZ> &queries)
{
    double totalTime = 0.0;

    // Prepare oneAPI cloud
    pcl::oneapi::Octree::PointCloud cloud_device;
    cloud_device.upload(points);

    // Upload queries and radiuses
    pcl::oneapi::Octree::Queries queries_device;
    queries_device.upload(queries);

    pcl::oneapi::NeighborIndices result_device(queries_device.size(), max_answers);

    // oneAPI build
    pcl::oneapi::Octree octree_device;
    octree_device.setCloud(cloud_device);
    octree_device.build();

    pcl::oneapi::Octree::ResultSqrDists sqr_distance;

    octree_device.approxNearestSearch(queries_device, result_device, sqr_distance);

    for (int i = 0; i < TEST_NUM; i++)
    {

        auto start = std::chrono::steady_clock::now();

        octree_device.approxNearestSearch(queries_device, result_device, sqr_distance);

        auto stop = std::chrono::steady_clock::now();
        auto ipp_time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count() / 1000.0;
        totalTime += ipp_time;

        printf("oneapi approx nearest took %.3f milliseconds \n", ipp_time);
    }
    printf("\n\nTotal average time %.3f milliseconds \n", totalTime/TEST_NUM );

    std::vector<int> downloaded;
    std::vector<float> dists_device_downloaded;
    result_device.data.download(downloaded);
    sqr_distance.download(dists_device_downloaded);

    printf("result size is %ld \n\n\n", downloaded.size());
}

void pcl_octree_approxNearestSearch(std::vector<pcl::PointXYZ> &points, std::vector<pcl::PointXYZ> &queries)
{
    double totalTime = 0.0;

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
        totalTime += ipp_time;

        printf("pcl approx nearest took %.3f milliseconds \n", ipp_time);
    }
    printf("\n\nTotal average time %.3f milliseconds \n", totalTime/TEST_NUM );

    printf("\n\n\n");
}

void oneapi_octree_radiusSearch(std::vector<pcl::PointXYZ> &points,
                                std::vector<pcl::PointXYZ> &queries,
                                std::vector<float> &radiuses)
{
    double totalTime = 0.0;

    // Prepare oneAPI cloud
    pcl::oneapi::Octree::PointCloud cloud_device;
    cloud_device.upload(points);

    // Upload queries and radiuses
    pcl::oneapi::Octree::Queries queries_device;
    pcl::oneapi::Octree::Radiuses radiuses_device;
    queries_device.upload(queries);
    radiuses_device.upload(radiuses);

    // oneAPI build
    pcl::oneapi::Octree octree_device;
    octree_device.setCloud(cloud_device);
    octree_device.build();

    pcl::oneapi::NeighborIndices result_device(queries_device.size(), max_answers);

    for (int i = 0; i < TEST_NUM; i++)
    {

        auto start = std::chrono::steady_clock::now();

        octree_device.radiusSearch(queries_device, radiuses_device, max_answers, result_device);

        auto stop = std::chrono::steady_clock::now();
        auto ipp_time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count() / 1000.0;


        printf("oneapi radius search took %.3f milliseconds \n", ipp_time);
        
        if(i==0)
            continue;
        totalTime += ipp_time;
    }
    printf("\n\nTotal average time %.3f milliseconds \n", totalTime/(TEST_NUM - 1) );

    std::vector<int> downloaded;
    // std::vector<float> dists_device_downloaded;
    result_device.data.download(downloaded);
    // sqr_distance.download(dists_device_downloaded);

    printf("result size is %ld \n\n\n", downloaded.size());
}

void pcl_octree_radiusSearch(std::vector<pcl::PointXYZ> &points,
                             std::vector<pcl::PointXYZ> &queries,
                             std::vector<float> &radiuses)
{
    double totalTime = 0.0;

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
        totalTime += ipp_time;

        printf("pcl radius search took %.3f milliseconds \n", ipp_time);
    }
    printf("\n\nTotal average time %.3f milliseconds \n", totalTime/TEST_NUM );

}


void display_device_info(cl::sycl::queue mQueue)
{
    int c_width = 24;
    auto device = mQueue.get_device();
    auto p_name = device.get_platform().get_info<sycl::info::platform::name>();
    std::cout << std::setw(c_width) << "Platform Name: " << p_name << "\n";

    auto p_version = device.get_platform().get_info<sycl::info::platform::version>();
    std::cout << std::setw(c_width) << "Platform Version: " << p_version << "\n";

    auto d_name = device.get_info<sycl::info::device::name>();
    std::cout << std::setw(c_width) << "Device Name: " << d_name << "\n";

    auto max_work_group = device.get_info<sycl::info::device::max_work_group_size>();
    std::cout << std::setw(c_width) << "Max Work Group Size: " << max_work_group << "\n";

    auto max_compute_units = device.get_info<sycl::info::device::max_compute_units>();
    std::cout << std::setw(c_width) << "Max Compute Units: " << max_compute_units << "\n";

    auto global_mem_size = device.get_info<sycl::info::device::global_mem_size>();
    std::cout << std::setw(c_width) << "Global Mem Size: " << global_mem_size << "\n";  // 13604175872  12GB

    auto local_mem_size = device.get_info<sycl::info::device::local_mem_size>();
    std::cout << std::setw(c_width) << "Local  Mem Size: " << local_mem_size << "\n";   // 65536  64KB

    std::cout << std::endl << std::endl;
}


int main(int argc, char **argv)
{

    sycl::queue q = dpct::get_default_queue();

    display_device_info(q);

    printf("oneapi pcl octree test\n");
    printf("point data  size is %10zu\n", data_size);
    printf("point query size is %10zu\n", query_size);
    printf("max radius  size is %9.1f\n", max_radius);

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

    oneapi_octree_approxNearestSearch(points, queries);
    pcl_octree_approxNearestSearch(points, queries);

    oneapi_octree_radiusSearch(points, queries, radiuses);

    pcl_octree_radiusSearch(points, queries, radiuses);
}