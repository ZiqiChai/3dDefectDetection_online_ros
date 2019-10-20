//just open a sensor, get one snapshot and close

//std c/c++
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

//GoSdk
#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <stdlib.h>

//OpenCV
#include <opencv2/opencv.hpp>

//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

//constants
#define SENSOR_IP "192.168.1.10"
#define RECEIVE_TIMEOUT 20000000
#define INVALID_RANGE_16BIT     ((signed short)0x8000)          // gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data.
#define DOUBLE_MAX              ((k64f)1.7976931348623157e+308) // 64-bit double - largest positive value.
#define INVALID_RANGE_DOUBLE    ((k64f)-DOUBLE_MAX)             // floating point value to represent invalid range data.

#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)

typedef struct
{
	double x;   // x-coordinate in engineering units (mm) - position along laser line
	double y;   // y-coordinate in engineering units (mm) - position along the direction of travel
	double z;   // z-coordinate in engineering units (mm) - height (at the given x position)
	unsigned char intensity;
}ProfilePoint;





//main
int main(int argc, char **argv)
{
	kStatus status;
	kAssembly api = kNULL;
	GoSystem system = kNULL;
	GoSensor sensor = kNULL;
	kIpAddress ipAddress;
	kChar model_name[50];
	GoDataSet dataset = kNULL;
	GoStamp *stamp = kNULL;
	GoProfilePositionX positionX = kNULL;
	GoDataMsg dataObj;
	GoMeasurementData *measurementData = kNULL;
	unsigned int i,j,k;

	//Hello message
	std::cout << "Gocator example_3 running" << std::endl;

	// construct Gocator API Library
	if ((status = GoSdk_Construct(&api)) != kOK)
	{
		std::cout << "Error: GoSdk_Construct: " << status << std::endl;
		return -1;
	}

	// construct GoSystem object
	if ((status = GoSystem_Construct(&system, kNULL)) != kOK)
	{
		std::cout << "Error: GoSystem_Construct: " << status << std::endl;
		return -1;
	}

	// obtain GoSensor object by sensor IP address
	kIpAddress_Parse(&ipAddress, SENSOR_IP);
	if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK)
	{
		std::cout << "Error: GoSystem_FindSensorByIpAddress: " << status << std::endl;
		std::cout
		<<"error code defined in kStatus :			 \n"
		<<"#define kERROR_STATE               (-1000)\n"
		<<"#define kERROR_NOT_FOUND           (-999) \n"
		<<"#define kERROR_COMMAND             (-998) \n"
		<<"#define kERROR_PARAMETER           (-997) \n"
		<<"#define kERROR_UNIMPLEMENTED       (-996) \n"
		<<"#define kERROR_MEMORY              (-994) \n"
		<<"#define kERROR_TIMEOUT             (-993) \n"
		<<"#define kERROR_INCOMPLETE          (-992) \n"
		<<"#define kERROR_STREAM              (-991) \n"
		<<"#define kERROR_CLOSED              (-990) \n"
		<<"#define kERROR_VERSION             (-989) \n"
		<<"#define kERROR_ABORT               (-988) \n"
		<<"#define kERROR_ALREADY_EXISTS      (-987) \n"
		<<"#define kERROR_NETWORK             (-986) \n"
		<<"#define kERROR_HEAP                (-985) \n"
		<<"#define kERROR_FORMAT              (-984) \n"
		<<"#define kERROR_READ_ONLY           (-983) \n"
		<<"#define kERROR_WRITE_ONLY          (-982) \n"
		<<"#define kERROR_BUSY                (-981) \n"
		<<"#define kERROR_CONFLICT            (-980) \n"
		<<"#define kERROR_OS                  (-979) \n"
		<<"#define kERROR_DEVICE              (-978) \n"
		<<"#define kERROR_FULL                (-977) \n"
		<<"#define kERROR_IN_PROGRESS         (-976) \n"
		<<"#define kERROR                     (0)	 \n"
		<<"#define kOK                        (1) 	 \n"
		<<std::endl;
		return -1;
	}

	// create connection to GoSensor object
	if ((status = GoSensor_Connect(sensor)) != kOK)
	{
		std::cout << "Error: GoSensor_Connect: " << status << std::endl;
		return -1;
	}

	// enable sensor data channel
	if ((status = GoSystem_EnableData(system, kTRUE)) != kOK)
	{
		std::cout << "Error: GoSensor_EnableData: " << status << std::endl;
		return -1;
	}

	//gets the sensor model
	if ((status = GoSensor_Model(sensor, model_name, 50)) != kOK )
	{
		std::cout << "Error: GoSensor_Model: " << status << std::endl;
		return -1;
	}

	//prints sensor info
	std::cout << "Connected to Sensor: " << std::endl;
	std::cout << "\tModel: \t" << model_name << std::endl;
	std::cout << "\tIP: \t" << SENSOR_IP << std::endl;
	std::cout << "\tSN: \t" << GoSensor_Id(sensor) << std::endl;
	std::cout << "\tState: \t" << GoSensor_State(sensor) << std::endl;


	while(cv::waitKey(10) != 'q')
	{


	// start Gocator sensor
		if ((status = GoSystem_Start(system)) != kOK)
		{
			std::cout << "Error: GoSystem_Start: " << status << std::endl;
			return -1;
		}

	//Get data
		std::cout << "\n\nSensor is running ..." << std::endl;
	// while(cv::waitKey(10) != 'q')
	// {
		if (GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT) == kOK)
		{
			short int* height_map_memory = NULL;
			unsigned char* intensity_image_memory = NULL;
			ProfilePoint **surfaceBuffer = NULL;
			k32u surfaceBufferHeight = 0;

			std::cout << "************************* Start of GoSystem_ReceiveData *************************" << std::endl;
			std::cout << "\nData message received: " << std::endl;
			std::cout << "Dataset total count: " << GoDataSet_Count(dataset) << "\n" << std::endl;

			// each result can have multiple data items
			// loop through all items in result message
			for (i = 0; i < GoDataSet_Count(dataset); ++i)
			{
				std::cout << "\n\nDataset index count: " << i << std::endl;
				dataObj = GoDataSet_At(dataset, i);
				switch(GoDataMsg_Type(dataObj))
				{



					case GO_DATA_MESSAGE_TYPE_STAMP:
					{
						GoStampMsg stampMsg = dataObj;
						std::cout << "Stamp Message batch count: " << GoStampMsg_Count(stampMsg) << std::endl;
						for (j = 0; j < GoStampMsg_Count(stampMsg); ++j)
						{
							stamp = GoStampMsg_At(stampMsg, j);
							std::cout << "  Timestamp: " << stamp->timestamp << std::endl;
							std::cout << "  Encoder: " << stamp->encoder << std::endl;
							std::cout << "  Frame index: " << stamp->frameIndex << std::endl;
						}
					}
					break;



					case GO_DATA_MESSAGE_TYPE_MEASUREMENT:
					{
						GoMeasurementMsg measurementMsg = dataObj;
						std::cout << "Measurement Message batch count: " << GoMeasurementMsg_Count(measurementMsg) << std::endl;
						for (k = 0; k < GoMeasurementMsg_Count(measurementMsg); ++k)
						{
							measurementData = GoMeasurementMsg_At(measurementMsg, k);
							std::cout << "Measurement ID: " << GoMeasurementMsg_Id(measurementMsg) << std::endl;
							std::cout << "Measurement Value: " << measurementData->value << std::endl;
							std::cout << "Measurement Decision: " << measurementData->decision << std::endl;
						}
					}
					break;



					case GO_DATA_MESSAGE_TYPE_SURFACE:
					{
						//point cloud
						pcl::PointCloud<pcl::PointXYZ> _p_cloud;
						//visualization window
						pcl::visualization::PCLVisualizer viewer_("Gocator3109 Snapshot");
						//cast to GoSurfaceMsg
						GoSurfaceMsg surfaceMsg = dataObj;
						//Get general data of the surface
						unsigned int row_count = GoSurfaceMsg_Length(surfaceMsg);
						unsigned int width = GoSurfaceMsg_Width(surfaceMsg);
						unsigned int exposure = GoSurfaceMsg_Exposure(surfaceMsg);
						//get offsets and resolutions
						double xResolution = NM_TO_MM(GoSurfaceMsg_XResolution(surfaceMsg));
						double yResolution = NM_TO_MM(GoSurfaceMsg_YResolution(surfaceMsg));
						double zResolution = NM_TO_MM(GoSurfaceMsg_ZResolution(surfaceMsg));
						double xOffset = UM_TO_MM(GoSurfaceMsg_XOffset(surfaceMsg));
						double yOffset = UM_TO_MM(GoSurfaceMsg_YOffset(surfaceMsg));
						double zOffset = UM_TO_MM(GoSurfaceMsg_ZOffset(surfaceMsg));
						/*
						//Print raw cloud metadata
						std::cout << "Surface Message" << std::endl;
						std::cout << "\tLength: " <<  row_count << std::endl;
						std::cout << "\tWidth: " << width << std::endl;
						std::cout << "\tExposure: " << exposure << std::endl;
						std::cout << "\tzOffset: " << zOffset << std::endl;
						*/

						//resize the point cloud
						_p_cloud.height = row_count;
						_p_cloud.width = width;
						_p_cloud.resize(row_count*width);

						//run over all rows
						for (unsigned int ii = 0; ii < row_count; ii++)
						{
							//get the pointer to row
							short *data = GoSurfaceMsg_RowAt(surfaceMsg,ii);

							//run over the width of row ii
							for (unsigned int jj = 0; jj < width; jj++)
							{
								//set xy in meters. x component inverted to fulfill right-handed frame (Gocator is left-handed!)
								_p_cloud.points.at(ii*width+jj).x = -0.001*(xOffset + xResolution*jj);
								_p_cloud.points.at(ii*width+jj).y = 0.001*(yOffset + yResolution*ii);

								//set z  in meters.
								if (data[jj] != INVALID_RANGE_16BIT )
									_p_cloud.points.at(ii*width+jj).z = 0.001*(zOffset + zResolution*data[jj]);
								else
									_p_cloud.points.at(ii*width+jj).z = 0.001*(INVALID_RANGE_DOUBLE);
							}
						}
						pcl::io::savePCDFileASCII("PointCloudGpcator.pcd",_p_cloud);
						pcl::io::savePLYFileASCII("PointCloudGpcator.ply",_p_cloud);

					}
					break;



					case GO_DATA_MESSAGE_TYPE_VIDEO:
					{
						//cast to GoVideoMsg
						GoVideoMsg videoMsg = dataObj;

						//Get general data of the image
						unsigned int CameraIndex = GoVideoMsg_CameraIndex(videoMsg);
						unsigned int Width = GoVideoMsg_Width(videoMsg);
						unsigned int Height = GoVideoMsg_Height(videoMsg);
						unsigned int ExposureIndex = GoVideoMsg_ExposureIndex(videoMsg);
						unsigned int Exposure = GoVideoMsg_Exposure(videoMsg);

						//Print raw image metadata
						std::cout << "Video Message Received!"<< std::endl;
						std::cout << "\tCameraIndex: [" << CameraIndex <<"] !"<< std::endl;
						std::cout << "\t\tWidth & Height: [" << Width <<"] ["<< Height <<"] !"<< std::endl;
						std::cout << "\t\tExposureIndex: [" << ExposureIndex <<"] !"<< std::endl;
						std::cout << "\t\tExposure: [" << Exposure <<"] ns!"<< std::endl;

						// GoVideoMsg_Source(videoMsg);
						// GoVideoMsg_Cfa(videoMsg);

						switch(GoVideoMsg_PixelFormat(videoMsg))
						{
							case kPIXEL_FORMAT_NULL:
							{
								std::cout << "\t\tkPIXEL_FORMAT_NULL kPIXEL_FORMAT Received!"<< std::endl;
								std::cout<<"break kPIXEL_FORMAT_NULL kPIXEL_FORMAT Received!\n"<<std::endl;
							}
							break;

							case kPIXEL_FORMAT_8BPP_GREYSCALE:
							{
								std::cout << "\t\tkPIXEL_FORMAT_8BPP_GREYSCALE kPIXEL_FORMAT Received!"<< std::endl;
								std::cout << "\t\t8-bit greyscale (k8u)!"<< std::endl;
								cv::Mat temp_img(Height+100, Width+100, CV_8UC1);

								//run over all rows
								for (unsigned int ii = 0; ii < Height; ii++)
								{
									std::cout<<ii<<" ";
									//get the pointer to row
									short *data =NULL;
									data = (short*) GoVideoMsg_RowAt(videoMsg,ii);
									// //run over the width of row ii
									for (unsigned int jj = 0; jj < Width; jj++)
									{
										if (data[jj] != INVALID_RANGE_16BIT )
											temp_img.at<short>(ii,jj) = data[jj];
										else
											temp_img.at<short>(ii,jj) = 0;
										// std::cout<<ii<<" "<<jj<<" "<<data[jj]<<" "<< INVALID_RANGE_16BIT <<std::endl;
									}
								}
								cv::Mat imageROI= temp_img(cv::Range(0,Height), cv::Range(0,Width));

								if (CameraIndex == 0)
								{
									std::cout<<"This is Camera index [0], image from Front Camera [Left Camera]!"<<std::endl;
									cv::namedWindow("Camera 0",cv::WINDOW_NORMAL);
									cv::imshow("Camera 0",imageROI);
									cv::waitKey(50);
									cv::imwrite("Camera 0 left.jpg",imageROI);
								}
								if (CameraIndex == 1)
								{
									std::cout<<"This is Camera index [1], image from Back Camera [right Camera]!"<<std::endl;
									cv::namedWindow("Camera 1",cv::WINDOW_NORMAL);
									cv::imshow("Camera 1",imageROI);
									cv::waitKey(50);
									cv::imwrite("Camera 1 right.jpg",imageROI);
								}
								std::cout<<"break kPIXEL_FORMAT_8BPP_GREYSCALE!\n"<<std::endl;
							}
							break;

							case kPIXEL_FORMAT_8BPP_CFA:
							{
								std::cout << "\t\tkPIXEL_FORMAT_8BPP_CFA kPIXEL_FORMAT Received!"<< std::endl;
								std::cout << "\t\t8-bit color filter array (k8u)!"<< std::endl;
								std::cout<<"break kPIXEL_FORMAT_8BPP_CFA!\n"<<std::endl;
							}
							break;

							case kPIXEL_FORMAT_8BPC_BGRX:
							{
								std::cout << "\t\tkPIXEL_FORMAT_8BPC_BGRX kPIXEL_FORMAT Received!"<< std::endl;
								std::cout << "\t\t8-bits-per-channel color with 4 channels (blue/green/red/unused)(kRgb)!"<< std::endl;
								std::cout<<"break kPIXEL_FORMAT_8BPC_BGRX!\n"<<std::endl;
							}
							break;

							case kPIXEL_FORMAT_1BPP_GREYSCALE:
							{
								std::cout << "\t\tkPIXEL_FORMAT_1BPP_GREYSCALE kPIXEL_FORMAT Received!"<< std::endl;
								std::cout << "\t\t1-bit greyscale, 8 packed pixels per image element (k8u)!"<< std::endl;
								std::cout<<"break kPIXEL_FORMAT_1BPP_GREYSCALE!\n"<<std::endl;
							}
							break;
						}
					}
					break;
				}
			}
			GoDestroy(dataset);
			std::cout << "************************* End of GoSystem_ReceiveData ***************************\n\n" << std::endl;
		}
		else
		{
			printf ("Error: No data received during the waiting period\n");
		}
	// }

	// stop Gocator sensor
		if ((status = GoSystem_Stop(system)) != kOK)
		{
			std::cout << "Error: GoSystem_Stop: " << status << std::endl;
			return -1;
		}
	}

	// destroy handles
	GoDestroy(system);
	GoDestroy(api);

	//bye bye message
	std::cout << "Sensor Stopped. Program finished !" << std::endl;
	return 1;
}