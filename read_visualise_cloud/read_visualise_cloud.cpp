/**********************************************************************
* Simple XYZ point cloud 3D visualiser of a .pcd file using PCL libraries
* The algorithm requires a .pcd file passed 
*
* The MIT License (MIT)
* Copyright (c) 2019 Felip Martí
* Swinburne University of Technology
* https://felipmarti.github.io
*
**********************************************************************/


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


/*
 *  Main
 **/

int main (int argc, char** argv)
{

    // Check input
    if (argc != 2) {
        printf("Usage: %s FILE.pcd\n",argv[0]);
        exit(1);
    }
    const char* file = argv[1];

    // Reading file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloudPtr) == -1) // Load the file into the cloud pointer
    {
        PCL_ERROR ("Couldn't read file: %s \n",file);
        return (-1);
    }

    // Visualising cloud
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (file));   // Visualiser with the file name
    viewer->setBackgroundColor (0, 0, 0);                   // RGB 000->Black
    viewer->addPointCloud<pcl::PointXYZ> (cloudPtr, file);  // Cloud pointer and the name of the cloud
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, file);
    viewer->initCameraParameters ();
    viewer->setCameraPosition(0, 0, 8000,    0, 0, 0,   0, 0, 0,  0);

    // Wait so the user can contemplate the amazing cloud
    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);
    }

}

