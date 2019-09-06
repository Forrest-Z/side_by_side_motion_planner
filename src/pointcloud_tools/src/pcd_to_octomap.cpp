#include <iostream>
#include <pcl/io/pcd_io.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <fstream>
#include <sstream>

void pc_to_ot(
    pcl::PointCloud<pcl::PointXYZ>& cloud,
    octomap::OcTree&                ot) 
{
    for (pcl::PointCloud<pcl::PointXYZ>::iterator it=cloud.begin();
        it!=cloud.end();
        it++)
    {
        ot.updateNode(octomap::point3d(it->x, it->y, it->z), true, true);
    }

    ot.toMaxLikelihood();
    ot.prune();
}

int main(int argc, char** argv) 
{
    if (argc < 3){
        std::cerr << "please provide resolution and path to point cloud file" << std::endl;
        return 1;
    }
    float res = atof(argv[1]);
    std::string file = argv[2];

    std::cout << std::endl;
    std::cout << "Map Resolution: " << res << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::string ext = file.substr(file.find_last_of(".") +1);
    if (ext == "pcd") {
        std::cout << "PCD File: " << file << std::endl;
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloud) == -1) //* load the file
        {
            std::cerr << "cannot open pcd file " << file << std::endl;
        }
    }
    else if (ext == "pts") {
        std::cout << "Points File: " << file << std::endl;
        std::ifstream infile(file.c_str());
        std::string line;
        while (std::getline(infile, line))
        {
            std::istringstream iss(line);
            float x, y, z;
            if (!(iss >> x >> y >> z)) { 
                std::cerr << "could not extract x y z from file" << std::endl; 
                return 1;
            }
            else{
                pcl::PointXYZ p;
                p.x = x;
                p.y = y;
                p.z = z;
                cloud->points.push_back(p);
            }
        }
        cloud->is_dense = true;
        cloud->width = cloud->points.size();
        cloud->height = 1.0;
    }
    else {
        std::cerr << "cannot open file of extension " << ext << std::endl;
    }
    std::cout << "Using " << cloud->points.size()  << " points" << std::endl;

    octomap::OcTree ot(res);
    pc_to_ot(*cloud, ot);

    std::cout << "OcTree has " << ot.size()  << " nodes" << std::endl;

    std::string out_file_name = file.substr(0, file.length() - 4) + ".bt";
    std::cerr << "Saving to file: " << out_file_name << std::endl;
    ot.writeBinary(out_file_name);
    std::cerr <<"Saved.\n" << std::endl;

    return 0;
}