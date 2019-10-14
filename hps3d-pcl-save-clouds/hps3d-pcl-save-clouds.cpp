/**********************************************************************
* Modification of the sample code from Hypersen Technology, Inc.
* to include PCL - point cloud library http://www.pointclouds.org
*
* This example saves the 3D point cloud of all the HPS3D-160 cameras 
* connected, and saves all the clouds in the PATH entered.
* The algorithm requires a PATH to save the clouds, the NUMBER_OF_THE_SCENE, 
* and a TIMESTAMP.
*
* The MIT License (MIT)
* Copyright (c) 2019 Felip Martí
* Swinburne University of Technology
* https://felipmarti.github.io
*
* Copyright (c) 2018, Hypersen Technology, Inc. 2018
* https://github.com/hypersen/HPS3D_SDK
* Kevin
* Kevin_Wang@hypersen.com
*
**********************************************************************/

#include <iostream>
#include <signal.h>
#include "api.h"

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>



HPS3D_HandleTypeDef handle[DEV_NUM];    // Array of handlers to connect multiple devices
AsyncIObserver_t My_Observer[DEV_NUM];  // Array of observers to configure different devices
uint8_t connect_number = 0;             // Number of the devices connected.


/*
 * Debugging use
 * */
void User_Printf(char *str)
{
    printf("%s\n",str);
}


/*
 * Function to remove the device when closing
 * */
void signal_handler(int signo)
{
    for(int i = 0;i<connect_number;i++)
    {
        HPS3D_RemoveDevice(&handle[i]);
        printf("Removing device %d before closing\n",i);
    }
    exit(0);
}


/*
 * Main
 * */
int main(int argc, char *argv[])
{

    // Check input
    if (argc != 4) {
        printf("Usage: %s PATH SCENE_NUMBER TIMESTAMP\n",argv[0]);
        exit(1);
    }
    const char* path = argv[1];
    const unsigned int sceneNumber = atoi(argv[2]);
    const unsigned long timestamp = atoi(argv[3]);


    int initOK = 1;                 // Variable for the endless loop 
    int typeConnection  = 0;        // This is the type of connection 0 USB, 1 Ethernet
    RET_StatusTypeDef ret = RET_OK; // Used for the device connection function? Only Ethernet uses ret variable


    // Debug stuff
    HPS3D_SetDebugEnable(false);   
    HPS3D_SetDebugFunc(&User_Printf);


    // How to close the program
    if(signal(SIGINT,signal_handler) == SIG_ERR) {
        printf("sigint error");
        return 1;
    }


    // Connexion via USB
    //printf("select transport type:(0:USB 1:Ethernet)\n");
    //scanf("%d",&typeConnection);
    typeConnection=0; // We use USB, so directly I choose USB

    // If Ethernet Connection (not USB). Not tested by Felip
    if(typeConnection == 1)
    {
        /*set server IP*/
        ret = HPS3D_SetEthernetServerInfo(&handle[0],"192.168.0.10",12345);
        if(ret != RET_OK)
        {
            printf("HPS3D_SetEthernetServerInfo error ,ret = %d\n",ret);
            initOK = 0;
            return 1;
        }    
    }


    // Return value of the number of devices successfully connected
    connect_number = HPS3D_AutoConnectAndInitConfigDevice(handle);
    printf("Devices successfully connected = %d\n",connect_number);
    if(connect_number == 0)
    {
        // 0 devices found
        printf("I think no devices have been found, so we close the program. \n");
        initOK = 0;
        return 1;
    }


    //Init HPS3d handles and configuration for devices
    for(int i = 0;i<connect_number;i++)
    {
        printf("HPS3D-160 camera number: %d \n",i);
        // TODO: Trying to get unique ID for each device
        printf("    Device name %s \n",handle[i].DeviceName);
        printf("    Device address %d \n",handle[i].DeviceAddr);
        Version_t version_t;
        HPS3D_GetDeviceVersion(&handle[i], &version_t);
        printf("    Device year %d, month %d, day %d, major %d, minor %d, rev %d \n",version_t.year,version_t.month,version_t.day,version_t.major,version_t.minor,version_t.rev);

        // Before enabling the point cloud data, ensure that the optical compensation enable is enabled, otherwise the correct point cloud result cannot be obtained.
        HPS3D_SetOpticalEnable(&handle[i],true);

        // Sets single shot 
        handle[i].RunMode = RUN_SINGLE_SHOT;

        // Sets running mode
        HPS3D_SetRunMode(&handle[i]);
    }
    // Add this at the end. Enables point cloud output 
    HPS3D_SetPointCloudEn(true);


    // Init clouds 
    std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr > bunchOfCloudsPtr;
    bunchOfCloudsPtr.resize(connect_number);
    for (int i=0;i<connect_number;i++) {
        bunchOfCloudsPtr[i]=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }


    // Capture data from all devices connected
    for (int device=0;device<connect_number;device++) {

        ret = HPS3D_SingleMeasurement(&handle[device]);
        if(ret == RET_OK ) {
    
            switch(handle[device].RetPacketType)
            {
                case SIMPLE_ROI_PACKET :
                    printf("Simple Roi measure distance average:%d\n",handle[device].MeasureData.simple_roi_data[0].distance_average);
                    break;
                case FULL_ROI_PACKET :
                    printf("Full Roi measure distance average:%d\n",handle[device].MeasureData.full_roi_data[0].distance_average);
                    break;
                case FULL_DEPTH_PACKET : {
                    printf("Full depth measure distance average:%d\n",handle[device].MeasureData.full_depth_data->distance_average);

                    // Fill the PCL cloud parameters 
                    bunchOfCloudsPtr[device]->width = handle[device].MeasureData.point_cloud_data->width;
                    bunchOfCloudsPtr[device]->height = handle[device].MeasureData.point_cloud_data->height;
                    // Specifies if all the data in points is finite (true), or whether the XYZ values of certain points might contain Inf/NaN values (false).
                    bunchOfCloudsPtr[device]->is_dense = false; 
                    bunchOfCloudsPtr[device]->points.resize(bunchOfCloudsPtr[device]->width * bunchOfCloudsPtr[device]->height);
                    
                    // Fill the PCL cloud data 
                    for (size_t j = 0; j < bunchOfCloudsPtr[device]->points.size (); ++j){
                        bunchOfCloudsPtr[device]->points[j].x = handle[device].MeasureData.point_cloud_data->point_data[j].x;
                        bunchOfCloudsPtr[device]->points[j].y = handle[device].MeasureData.point_cloud_data->point_data[j].y;
                        bunchOfCloudsPtr[device]->points[j].z = handle[device].MeasureData.point_cloud_data->point_data[j].z;
                    }

                    // Save cloud to file
                    char filename[100];
                    sprintf(filename, "%s/%05d-HPS3D-%02d-%lu.pcd",path,sceneNumber,device + 1,timestamp);
                    pcl::io::savePCDFileASCII(filename, *bunchOfCloudsPtr[device]);
                    std::cerr << "Saved " << bunchOfCloudsPtr[device]->points.size () << " data points to test_pcd.pcd." << std::endl;

                    }
                    break;
                case SIMPLE_DEPTH_PACKET :
                    printf("simple depth measure distance average:%d\n",handle[device].MeasureData.simple_depth_data->distance_average);
                    break;
                case NULL_PACKET :
                    printf("return packet is null\n");
                    break;
                default:
                    printf("system error\n");
                    break;
            }

        }

    }
    
    return 0;
}

