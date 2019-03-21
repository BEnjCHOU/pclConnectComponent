#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <thread>
//#include <vector>
#include <pcl/point_types.h>
#include <boost/asio.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/search.h>
//#include <pcl/segmentation/segment_differences.h>
#include <pcl/filters/radius_outlier_removal.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
// write file
#include <fstream>
// save file number
int file_number = 0;
// start frame
int START_FRAME = 1;
const int END_FRAME = 1000;
//const double RADIUS = 1.5;
const double RADIUS_CONDITION = 0.2; // 1.5m
const int MIN_CLUSTER_POINTS = 100;
const int MAX_CLUSTER_POINTS = 400;
//output string to send
std::string output;
// connection
bool connected = false;

std::string read_data(boost::asio::ip::tcp::socket & socket)
{
    boost::asio::streambuf buf;
    boost::asio::read_until( socket, buf, "\n");
    // boost::system::error_code error;
    // boost::asio::read(socket, buf, boost::asio::transfer_all(), error);
    std::string data = boost::asio::buffer_cast<const char*>(buf.data());
    return data;
}

void send_data(boost::asio::ip::tcp::socket & socket, const std::string& message)
{
    //const std::string msg = message + "\n";
    boost::asio::write( socket, boost::asio::buffer(message) );
}

void server_start()
{
    // server connection and sender
    // create io instance
    boost::asio::io_context _io_service;
    // create listener for new connection
    boost::asio::ip::tcp::acceptor _acceptor(_io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 8080));
    // create socket
    boost::asio::ip::tcp::socket _socket(_io_service);
    // waiting for connection
    _acceptor.accept(_socket);
    while(1)
    {
        connected = true;
        if(output.length() > 0)
        {
            send_data(_socket, output);
            output = "";
        }
    }
}
/*void PclVisualization(pcl::PointCloud<pcl::PointXYZ>::Ptr DefaultData (new pcl::PointCloud<pcl::PointXYZ>);)
{
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(DefaultData,0,255,0);
    pcl::visualization::PCLVisualizer vis("3D View");
    vis.addPointCloud(DefaultData,green,"frontground",0);
    //vis.addCoordinateSystem(100);
    //OpenWindow
    while(!vis.wasStopped())
    {
        vis.spinOnce();
    }
}*/
std::string ClusterGrouping(int frame)
{
    std::string source = "../data_new2/frame-";
    std::string file_type = ".txt";
    std::string file = source + std::to_string(frame) + file_type;
    std::ifstream infile(file);
    // pcl statements
    pcl::PointCloud<pcl::PointXYZ>::Ptr DefaultData (new pcl::PointCloud<pcl::PointXYZ>);
    // read file
    for(std::string line; std::getline(infile, line);)
    {
        // create structure
        pcl::PointXYZ Points;
        std::istringstream in (line);
        in >> Points.x >> Points.y >> Points.z;
        DefaultData->push_back(Points);
    }
    // put all the cluster index in it, example: cluster_indices[0] contains all the index for the first cluster
    std::vector<pcl::PointIndices> cluster_indices;
    // kdTree search statement
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    // input cloud to kdtree for searching
    tree->setInputCloud (DefaultData);
    // cluster extraction statement
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // cluster conditions
    ec.setClusterTolerance (RADIUS_CONDITION);
    ec.setMinClusterSize (MIN_CLUSTER_POINTS);
    ec.setMaxClusterSize (MAX_CLUSTER_POINTS);
    ec.setSearchMethod (tree);
    ec.setInputCloud (DefaultData);
    ec.extract (cluster_indices);
    /* calculate center position of every cluster*/
    // count number of cluster
    int cluster_counter = 0;
    // string stream statement to put data in string stream for sending
    std::stringstream ss;
    // retrieve data by indices(index)
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        // point average statements
        float GroupCenterLocation_x = 0.0;
        float GroupCenterLocation_y = 0.0;
        float GroupCenterLocation_z = 0.0;
        // calculate center position of the group
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        // add every point then get average
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            GroupCenterLocation_x += DefaultData->points[*pit].x;
            GroupCenterLocation_y += DefaultData->points[*pit].y;
            GroupCenterLocation_z += DefaultData->points[*pit].z;
        }
        //cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        //cloud_cluster->width = cloud_cluster->points.size ();
        //cloud_cluster->height = 1;
        //cloud_cluster->is_dense = true;
        int IndicesFound = it->indices.size();
        // put data in string stream for sending
        ss << std::to_string(GroupCenterLocation_x/(float)IndicesFound) + " " + std::to_string(GroupCenterLocation_y/(float)IndicesFound) + " " + std::to_string(GroupCenterLocation_z/(float)IndicesFound) + " ";
        cluster_counter++;
    }
    std::string GroupedCenter = std::to_string(cluster_counter) + " " + ss.str() + "\n";
    return GroupedCenter;
}
//write to file
/*std::ofstream myfile;
 std::string filename = "../target/target-";
 std::string filetype = ".txt";
 std::string filewrite = filename + std::to_string(file_number) + filetype;
 myfile.open (filewrite);
 std::string writedata = std::to_string(cluster_counter) + " " + ss.str();
 myfile << writedata;
 myfile.close();
 file_number++;*/

int main()
{
    //std::thread sender (server_start) ;
    //while (1)
    //{
        //if(connected == true)
        {
            for(START_FRAME ; START_FRAME < END_FRAME+1 ; START_FRAME++)
            {
                //write
                std::ofstream myfile;
                std::string filename = "../target/target-";
                std::string filetype = ".txt";
                std::string filewrite = filename + std::to_string(file_number) + filetype;
                myfile.open (filewrite);
                
                output = ClusterGrouping(START_FRAME);
                myfile << output;
                myfile.close();
                std::cout << output << std::endl;
                file_number++;
            }
        }
    //}
    return 0;
}
