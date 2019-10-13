/**********************************************************************
* Modification of the Sample code from Hypersen Technology, Inc.
* to include PCL - point cloud library http://www.pointclouds.org
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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>


HPS3D_HandleTypeDef handle[DEV_NUM];    // Array of handles to connect multiple devices
AsyncIObserver_t My_Observer[DEV_NUM];  // Array of observers to configure different devices
uint8_t connect_number = 0;             // Number of the devices connected.


// Initialisation of PCL stuff: cloud, pointer to cloud, and pointer to visualiser
// TODO visualise clouds for diff devices, so array of clouds?
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));


/*
 * User processing function,Continuous measurement or asynchronous mode
 * in which the observer notifies the callback function
 * */
void* User_Func(HPS3D_HandleTypeDef *handle,AsyncIObserver_t *event)
{

	// printf("[User_Func] The type of package we are receiving is = %d\n",event->RetPacketType);

	switch(event->RetPacketType)
	{
		case SIMPLE_ROI_PACKET:
			printf("observer id = %d, distance average:%d\n",event->ObserverID,event->MeasureData.simple_roi_data[0].distance_average);
			break;
		case FULL_ROI_PACKET:
			printf("observer id = %d, distance average:%d\n",event->ObserverID,event->MeasureData.full_roi_data[0].distance_average);
			break;
		case FULL_DEPTH_PACKET:
			//printf("observer id = %d, distance average:%d\n",event->ObserverID,event->MeasureData.full_depth_data->distance_average);

            // Fill the PCL cloud parameters 
            cloud.width = event->MeasureData.point_cloud_data->width;
            cloud.height = event->MeasureData.point_cloud_data->height;
            cloud.is_dense = false; // Specifies if all the data in points is finite (true), or whether the XYZ values of certain points might contain Inf/NaN values (false).
            cloud.points.resize(cloud.width * cloud.height);
            
            // For testing
            //printf("observer id = %d, width x height = %d x %d = %d\n", event->ObserverID,cloud.width,cloud.height,cloud.width*cloud.height);

            // Fill the PCL cloud data 
            for (size_t i = 0; i < cloud.points.size (); ++i){
                event->ObserverID;
                cloud.points[i].x = event->MeasureData.point_cloud_data->point_data[i].x;
    			cloud.points[i].y = event->MeasureData.point_cloud_data->point_data[i].y;
                cloud.points[i].z = event->MeasureData.point_cloud_data->point_data[i].z;
            }
            
            *cloud_ptr=cloud;

			break;
		case SIMPLE_DEPTH_PACKET:
			printf("observer id = %d, distance average:%d\n",event->ObserverID,event->MeasureData.simple_depth_data->distance_average);
			break;
		case OBSTACLE_PACKET:
			printf("observer id = %d\n",event->ObserverID);
			printf(" Obstacle ID：%d\n",event->MeasureData.Obstacle_data->Id);
			printf(" LeftPoint:(%f,%f,%f)\n",event->MeasureData.Obstacle_data->LeftPoint.x,event->MeasureData.Obstacle_data->LeftPoint.y,event->MeasureData.Obstacle_data->LeftPoint.z);
			printf(" RightPoint:(%f,%f,%f)\n",event->MeasureData.Obstacle_data->RightPoint.x,event->MeasureData.Obstacle_data->RightPoint.y,event->MeasureData.Obstacle_data->RightPoint.z);
			printf(" UpperPoint:(%f,%f,%f)\n",event->MeasureData.Obstacle_data->UpperPoint.x,event->MeasureData.Obstacle_data->UpperPoint.y,event->MeasureData.Obstacle_data->UpperPoint.z);
			printf(" UnderPoint:(%f,%f,%f)\n",event->MeasureData.Obstacle_data->UnderPoint.x,event->MeasureData.Obstacle_data->UnderPoint.y,event->MeasureData.Obstacle_data->UnderPoint.z);
			printf(" MinPoint:(%f,%f,%f)\n",event->MeasureData.Obstacle_data->MinPoint.x,event->MeasureData.Obstacle_data->MinPoint.y,event->MeasureData.Obstacle_data->MinPoint.z);
			printf(" DistanceAverage:%d\n\n",event->MeasureData.Obstacle_data->DistanceAverage);
			break;
		case NULL_PACKET:
			printf(" packet is null\n");
			break;
		default:
			printf(" system error\n");
			break;
	}
}


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
    //TODO for diff devices, close them all.
	HPS3D_RemoveDevice(handle);
    printf("Removing device before closing\n");
    exit(0);
}


/*
 * Main
 * */
int main()
{

	int initOK = 1;                 // Variable for the endless loop 
	int typeConnection  = 0;        // This is the type of connection 0 USB, 1 Ethernet
	RET_StatusTypeDef ret = RET_OK; // Used for the device connection function? Only Ethernet uses ret variable

	HPS3D_SetDebugEnable(false);    // Debug stuff
	HPS3D_SetDebugFunc(&User_Printf);

    // How to close the program
    if(signal(SIGINT,signal_handler) == SIG_ERR) {
        printf("sigint error");
        return 1;
    }

	printf("select transport type:(0:USB 1:Ethernet)\n");
	printf("Yeah nah, we use USB, so 0)\n");
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

	// Init Devices
    // Return value of the number of devices successfully connected
    // TODO: set a fix device ID
	connect_number = HPS3D_AutoConnectAndInitConfigDevice(handle);
	printf("Devices successfully connected = %d\n",connect_number);
	if(connect_number == 0)
	{
        // 0 devices found
	    printf("I think no devices have been found, so we close the program. \n");
	    initOK = 0;
        return 1;
	}

    // We loop for all different devices found
	for(int i = 0;i < connect_number;i++)
	{
        // Observer initialization
        // There are two types of measurement modes: continuous measurement and single measurement.
        // Continuous measurement is asynchronous notification mode
        // Single measurement is synchronous mode. Only needs to call the interface. The code is different, we won't use this
        // The continuous measurement data return will notify the callback function, and the measurement data is saved in the MeasureData in the event.
		My_Observer[i].AsyncEvent = ISubject_Event_DataRecvd;   // Asynchronous notification event for data reception
		My_Observer[i].NotifyEnable = true;                     // Enabling observer
		My_Observer[i].ObserverID = i;                          // Observer ID, same as device number
		My_Observer[i].RetPacketType = NULL_PACKET;             // Here we select the type of data/packet we want, probably: FULL_DEPTH_PACKET or FULL_ROI_PACKET 
        // The initialisation done above seems useless. The configuration below will rewrite to FULL_DEPTH_PACKET
	}

	//Adding asynchronous observers. Only valid in asynchronous or continuous measurement mode 
	for(int i = 0;i<connect_number;i++)
	{
        // Adding callback function for each device, and each configured observer
	    HPS3D_AddObserver(&User_Func,&handle[i],&My_Observer[i]);

        // Before enabling the point cloud data, ensure that the optical compensation enable is enabled, otherwise the correct point cloud result cannot be obtained.
		HPS3D_SetOpticalEnable(&handle[i],true);

		// Sets continuous measurement mode 
		handle[i].RunMode = RUN_CONTINUOUS;

        // Starts measuring
		HPS3D_SetRunMode(&handle[i]);
	}
    // Add this at the end. Enables point cloud output 
	HPS3D_SetPointCloudEn(true);

    // Init visualiser
    viewer->setBackgroundColor (0, 0, 0);   // RGB 000->Black
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 8000,    0, 0, 0,   0, 0, 0,  0);

    // Main loop to visualise and wait new clouds 
    while (!viewer->wasStopped () && initOK) {

        // If there is nothing to visualise, don't visualise
        if (cloud.points.size() == 0) {
            printf("Size cloud = %lu. Not visualising, if not memory size increases\n", cloud.points.size() );
        }
        else {
            // Visualising cloud
            // TODO Visualise for diff cameras

            // Clear the view
            viewer->removeAllShapes();
            viewer->removeAllPointClouds(); 
        
            // Show point cloud
            viewer->addPointCloud<pcl::PointXYZ> (cloud_ptr, "HPS3D camera");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "HPS3D camera");

        }
        viewer->spinOnce (50);
    }

	return 0;
}

