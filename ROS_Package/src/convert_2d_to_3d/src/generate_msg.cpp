#include <ros/ros.h>

// ros image processing
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ros pcl processing
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>

// 
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>


// C++ libraries
#include <iostream>
#include <vector>

// For Generate message
#include "convert_2d_to_3d/Result.h"
#include "convert_2d_to_3d/robot_pose.h"
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

// sub sampling
#include <pcl/conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>

// plane segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

// Marker
#include <visualization_msgs/Marker.h>
#include <cmath>

// Handling NaN value
#include <float.h>


#define PI           3.141592
#define RADIAN       (PI / 180.0)
#define DEGREE       (180.0 / PI)
#define RAD2DEG(Rad) (Rad * DEGREE)
#define DEG2RAD(Deg) (Deg * PI / 180.0)

#define MODE_FLAG_LP	  	0
#define MODE_FLAG_STRAIGHT  1
#define MODE_FLAG_STOP 		2
#define MODE_FLAG_ELEVATION 3
#define MODE_FLAG_BACK 		4
#define MODE_FLAG_END 		5
#define ROT_LEFT			10
#define ROT_RIGHT			11
#define GO_BACK			    12

using namespace std;

vector<double> mAF_x(10);	// Initialize vector size 10  with 0 values
vector<double> mAF_y(10);
vector<double> mAF_degree(10);

// Moving Average Filters
// Input : input data
// Output : Average value of vector(except 0 value)


// Global variables initialize
cv::Mat RGB_image;
typedef pcl::PointCloud<pcl::PointXYZRGB> VPointImage;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_data(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_pt_data(new pcl::PointCloud<pcl::PointXYZRGB>);
darknet_ros_msgs::BoundingBoxes m_gBoundingboxes;
convert_2d_to_3d::robot_pose m_robot_pose;
pcl::PointXYZRGB m_pt;
uint32_t shape = visualization_msgs::Marker::ARROW;

// degree initinalization
int Mode_Flag = 0;
double degree = 0.0;
double controlInputDegree = 0.0;

// previous robot pose value
double prev_robot_pose_x = 0.0;
double prev_robot_pose_y = 0.0;
double prev_robot_pose_theta = 0.0;

//unit vector 
double v0;
double v1;
double v2;
double v3;

// Target destination
double _distance = 1.2;

int m_noBoundingBoxCnt = 0;
int m_targetLocationCnt = 0;

int m_Straight_Cnt = 0;
int m_stop_Cnt		= 0;
int m_elevation_Cnt = 0;
int m_back_Cnt = 0;
int m_End_Cnt = 0;

int m_Straint_Time = 150 ; //2 second
int m_Stop_Time = 10;
int m_Elevation_Time = 100;
int m_Back_Time = 30;

darknet_ros_msgs::BoundingBoxes gBoundingboxes;

ros::Subscriber image_sub ;
ros::Subscriber bouding_box_point_cloud_sub;		//off
ros::Subscriber roi_point_cloud_sub;
ros::Subscriber pointcloud_sub_;
ros::Subscriber boundingboxes_sub_;	//off
ros::Publisher  pub_result_;	//off	
ros::Publisher  Pub_pt;	//off
ros::Publisher  Pub_pt_color;	//off
ros::Publisher  marker_pub;	
ros::Publisher  marker_pub_xz;	
ros::Publisher  marker_pub_center;	
ros::Publisher  pub_robot_pose;	
ros::Publisher  roi_pt__pub;
// ROS_INFO("[%s] initialized...", ros::this_node::getName().c_str

// Moving Average Filters
// Input : input data
// Output : Average value of vector(except 0 value)
double MovingAverageFilter_Y(double input){
	
	int cnt = 0;	// count for 0 value
	double sum = 0.0;

	mAF_y.erase(mAF_y.begin());
	mAF_y.push_back(input);

	for(int i = 0; i < 10; i++){
		if(mAF_y[i] == 0){
			cnt++;
		}
		else{
			sum += mAF_y[i];
		}
	}
	
	return sum / (10 - cnt);
}

// Moving Average Filters
// Input : input data
// Output : Average value of vector(except 0 value)
double MovingAverageFilter_X(double input){
	
	int cnt = 0;	// count for 0 value
	double sum = 0.0;

	mAF_x.erase(mAF_x.begin());
	mAF_x.push_back(input);

	for(int i = 0; i < 10; i++){
		if(mAF_x[i] == 0){
			cnt++;
		}
		else{
			sum += mAF_x[i];
		}
	}
	
	return sum / (10 - cnt);
}

// Moving Average Filters
// Input : input data
// Output : Average value of vector(except 0 value)
double MovingAverageFilter_degree(double input){
	
	int cnt = 0;	// count for 0 value
	double sum = 0.0;

	mAF_degree.erase(mAF_degree.begin());
	mAF_degree.push_back(input);

	for(int i = 0; i < 10; i++){
		if(mAF_degree[i] == 0){
			cnt++;
		}
		else{
			sum += mAF_degree[i];
		}
	}
	
	return sum / (10 - cnt);
}

// GetAngleBetweenTwoVectors
// Input : (x1,y1) (0,1) 
// Output : Angle of two vector (degree)
double GetAngleBetweenTwoVector(double dVec1X, double dVec1Y, double dVec2X, double dVec2Y)
{
	double dNum = ( dVec1X * dVec2X ) + ( dVec1Y * dVec2Y);
	double dDen = ( sqrt( pow( dVec1X, 2 ) + pow( dVec1Y, 2 ) ) * sqrt( pow( dVec2X, 2 ) + pow( dVec2Y, 2 ) ) );
	double dValue = RAD2DEG( acos( ( dNum / dDen ) ) );

	if(dVec1X > 0)
		return dValue;
	else
		return -1 * dValue;
}

// depth value -> orthographic projection XZ plane
// Input : x point, y point, z point
// Ouput : new z point
double projectionXYZ(double x, double y, double z)
{
	double x_value = abs(x);
	double y_value = abs(y);
	double z_value = abs(z);

	double new_z = sqrt(pow(z_value,2) - pow(y_value,2) - pow(x_value,2));
	
	return new_z;
}

// marker visualize - publish marker
// input : car license plate normal vector (a,b,c), car license plate center point(x,y)
void marker(double coEff0, double coEff1, double coEff2, double pointX, double pointY){
	
	// Marker
	visualization_msgs::Marker marker;
	visualization_msgs::Marker marker_xz;
	visualization_msgs::Marker marker_center;

	geometry_msgs::Point start_p, end_p;
	geometry_msgs::Point start_p1, end_p1;
	geometry_msgs::Point start_p2, end_p2;

	marker.header.frame_id = "camera_color_optical_frame";
	marker_xz.header.frame_id = "camera_color_optical_frame";
	marker_center.header.frame_id = "camera_color_optical_frame";

	// Set the marker type. 
	marker.type = shape;
	marker_xz.type = shape;
	marker_center.type = shape;
	
	// Set the marker action.
	marker.action = visualization_msgs::Marker::ADD;
	marker_xz.action = visualization_msgs::Marker::ADD;
	marker_center.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker
	start_p.x = 0.0;
	start_p.y = 0.0;
	start_p.z = 0.0;

	start_p1.x = 0.0;
	start_p1.y = 0.0;
	start_p1.z = 0.0;

	start_p2.x = 0.0;
	start_p2.y = 0.0;
	start_p2.z = 0.0;

	end_p.x = coEff0;
	end_p.y = coEff1;
	end_p.z = coEff2;

	end_p1.x = coEff0;
	end_p1.y = 0.0;
	end_p1.z = coEff2;

	end_p2.x = pointX;
	end_p2.y = 0.0;
	end_p2.z = pointY;

	marker.points.push_back(start_p);
	marker.points.push_back(end_p);

	marker_xz.points.push_back(start_p1);
	marker_xz.points.push_back(end_p1);

	marker_center.points.push_back(start_p2);
	marker_center.points.push_back(end_p2);

	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker_xz.pose.orientation.x = 0.0;
	marker_xz.pose.orientation.y = 0.0;
	marker_xz.pose.orientation.z = 0.0;
	marker_xz.pose.orientation.w = 1.0;

	marker_center.pose.orientation.x = 0.0;
	marker_center.pose.orientation.y = 0.0;
	marker_center.pose.orientation.z = 0.0;
	marker_center.pose.orientation.w = 1.0;
	

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.01;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;

	marker_xz.scale.x = 0.01;
	marker_xz.scale.y = 0.1;
	marker_xz.scale.z = 0.1;

	marker_center.scale.x = 0.01;
	marker_center.scale.y = 0.1;
	marker_center.scale.z = 0.1;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker_xz.color.r = 1.0f;
	marker_xz.color.g = 0.0f;
	marker_xz.color.b = 0.0f;
	marker_xz.color.a = 1.0;

	marker_center.color.r = 1.0f;
	marker_center.color.g = 0.0f;
	marker_center.color.b = 1.0f;
	marker_center.color.a = 1.0;

	marker_pub.publish(marker);
	marker_pub_xz.publish(marker_xz);
	marker_pub_center.publish(marker_center);
}

// Timer Call back
// For check the Mode
// Handing when you can not fine the bounding box
void timercallback1(const ros::TimerEvent&)
{
	// ROS_INFO("Callback 1 triggered");
	// std::cout << targetLocationCnt << std::endl;
	// Find Target location - Mode : MODE_FLAG_LP
	if(  (Mode_Flag == MODE_FLAG_LP)  && (abs(m_robot_pose.dis_x) < 0.1) && (abs(m_robot_pose.dis_y) < 0.1) && (abs(m_robot_pose.theta) < 10) && (m_robot_pose.dis_x != 0) && (m_robot_pose.dis_y != 0) && (m_robot_pose.theta != 0)  )
	{
		m_targetLocationCnt++;
		std::cout<< "m_targetLocationCnt : "<<m_targetLocationCnt<< std::endl; 
	// 	if(m_targetLocationCnt >= 50)
	// 		Mode_Flag = MODE_FLAG_STRAIGHT;
	}
	else{
		m_targetLocationCnt = 0;
	}

	//// No bounding box count

	if((Mode_Flag == MODE_FLAG_LP) && m_gBoundingboxes.bounding_boxes.empty()){
		m_noBoundingBoxCnt++;

		if(m_noBoundingBoxCnt >= 10){
			m_noBoundingBoxCnt = 0;
		
			std::cout<< "prev_robot_pose_x : " << prev_robot_pose_x << std::endl;
			std::cout<< "prev_robot_pose_y : " << prev_robot_pose_y << std::endl;
			std::cout<< "prev_robot_pose_theta : " << prev_robot_pose_theta << std::endl;
			// bounding box not detected, previous x,y,thetha information 
			// 1 . Too close -> go back
			if(abs(prev_robot_pose_x < 0.2)){
				m_robot_pose.dis_x = 0;
				m_robot_pose.dis_y = 0;
				m_robot_pose.theta = 0;
				m_robot_pose.mode = GO_BACK;

				pub_robot_pose.publish(m_robot_pose);
			}
			
			// 2. Too far -> .....
			// 3. Robot is too Left side -> Turn right
			else if((prev_robot_pose_y <0)&&(prev_robot_pose_theta <0)){
				m_robot_pose.dis_x = 0;
				m_robot_pose.dis_y = 0;
				m_robot_pose.theta = 0;
				m_robot_pose.mode = ROT_RIGHT;

				pub_robot_pose.publish(m_robot_pose);
			}
			else if((prev_robot_pose_y >0)&&(prev_robot_pose_theta <0)){
				m_robot_pose.dis_x = 0;
				m_robot_pose.dis_y = 0;
				m_robot_pose.theta = 0;
				m_robot_pose.mode = ROT_RIGHT;

				pub_robot_pose.publish(m_robot_pose);
			}
			// 4. Robot is too Right side -> Turn left 
			else if((prev_robot_pose_y <0)&&(prev_robot_pose_theta >0)){
				m_robot_pose.dis_x = 0;
				m_robot_pose.dis_y = 0;
				m_robot_pose.theta = 0;
				m_robot_pose.mode = ROT_LEFT;

				pub_robot_pose.publish(m_robot_pose);
			}
			else if((prev_robot_pose_y >0)&&(prev_robot_pose_theta >0)){
				m_robot_pose.dis_x = 0;
				m_robot_pose.dis_y = 0;
				m_robot_pose.theta = 0;
				m_robot_pose.mode = ROT_LEFT;

				pub_robot_pose.publish(m_robot_pose);
			}
			else{
				m_robot_pose.dis_x = 0;
				m_robot_pose.dis_y = 0;
				m_robot_pose.theta = 0;
				m_robot_pose.mode = 0;

				pub_robot_pose.publish(m_robot_pose);
			}
			
			
		}
	}

	// Mode Change + Count threshold time + publish robot pose
	// MODE_FLAG_STRAIGHT -> MODE_FLAG_STOP -> MODE_FLAG_ELEVATION -> MODE_FLAG_END
	switch(Mode_Flag){
		case MODE_FLAG_STRAIGHT:
			m_Straight_Cnt++;
			std::cout<< "m_Straight_Cnt : " << m_Straight_Cnt << std::endl;

			m_robot_pose.dis_x = 0;
			m_robot_pose.dis_y = 0;
			m_robot_pose.theta = 0;
			m_robot_pose.mode = MODE_FLAG_STRAIGHT;

			// std::cout<< "Mode_Flag (STOP): "  <<Mode_Flag<<std::endl;
			pub_robot_pose.publish(m_robot_pose);
			//	Mode change		
			// Mode_Flag = MODE_FLAG_STOP; 
		
			break;
		
		// case MODE_FLAG_STOP:
		// 	m_stop_Cnt++;
		// 	std::cout<< "m_stop_Cnt : " << m_stop_Cnt << std::endl;

		// 	m_robot_pose.dis_x = 0;
		// 	m_robot_pose.dis_y = 0;
		// 	m_robot_pose.theta = 0;
		// 	m_robot_pose.mode = MODE_FLAG_STOP;

		// 	// std::cout<< "Mode_Flag (ELEVATION): "  <<Mode_Flag<<std::endl;
		// 	pub_robot_pose.publish(m_robot_pose);

		// 	if(m_stop_Cnt >= m_Stop_Time)
		// 	{
		// 		Mode_Flag = MODE_FLAG_ELEVATION;
		// 		m_stop_Cnt = 0;
		// 	}
		// 	break;
		// case MODE_FLAG_ELEVATION:
		// 	m_elevation_Cnt++;
		// 	std::cout<< "m_elevation_Cnt : " << m_elevation_Cnt << std::endl;
		// 	m_robot_pose.dis_x = 0;
		// 	m_robot_pose.dis_y = 0;
		// 	m_robot_pose.theta = 0;
		// 	m_robot_pose.mode = MODE_FLAG_ELEVATION;

		// 	// std::cout<< "Mode_Flag (BACK): "  <<Mode_Flag<<std::endl;
		// 	pub_robot_pose.publish(m_robot_pose);

		// 	if(m_elevation_Cnt >= m_Elevation_Time)
		// 	{
		// 		Mode_Flag = MODE_FLAG_BACK;
		// 		m_elevation_Cnt = 0;
		// 	}
		// 	break;
		// case MODE_FLAG_END:
		// 	m_back_Cnt++;
		// 	std::cout<< "m_END_Cnt : " << m_End_Cnt << std::endl;
		// 	m_robot_pose.dis_x = 0;
		// 	m_robot_pose.dis_y = 0;
		// 	m_robot_pose.theta = 0;
		// 	m_robot_pose.mode = MODE_FLAG_BACK;

		// 	// std::cout<< "Mode_Flag (END): "  <<Mode_Flag<<std::endl;
		// 	pub_robot_pose.publish(m_robot_pose);

		// 	if(m_back_Cnt >= m_Back_Time)
		// 	{
		// 		Mode_Flag = MODE_FLAG_END;
		// 		m_back_Cnt = 0;
		// 	}
		// 	break;

	}

}

// Input : Output of yolo tiny v3 bounding box
// update the gBoundingboxes variable
void boundingbox_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& boundingbox) {
	m_noBoundingBoxCnt = 0;
	// std::cout<<"boundingbox_callback1"<<std::endl;
	gBoundingboxes.bounding_boxes.clear();
	for(int i = 0; i < boundingbox->bounding_boxes.size(); i++) {
		gBoundingboxes.bounding_boxes.push_back(boundingbox->bounding_boxes[i]);
		m_gBoundingboxes = gBoundingboxes;
	}
	// std::cout<<"boundingbox_callback2"<<std::endl;
}

// Input : Output of yolo tiny v3 bounding box
// Nothing just mode change
void boundingbox_callback2(const darknet_ros_msgs::BoundingBoxesConstPtr& boundingbox)
{
	//do nothing;
	//std::cout<< "GAE SSIBAL" << std::endl;


}

// Input : Image
// update RGB_image variable - cameraCallback flag 0,0,0 publish
void cameraCallback(const sensor_msgs::ImageConstPtr& msg){
	// std::cout<<"cameracallback1"<<std::endl;
	cv_bridge::CvImagePtr cam_image;
	cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	//std::cout<< "cameraCallback : "<< noBoundingBoxCnt <<std::endl;
	RGB_image = cam_image->image;

	
	// std::cout<<"cameracallback2"<<std::endl;
	// if there is no bounding box and count call back over 10

	//core dump problem here 

	// std::cout<<"cameracallback2"<<std::endl;

	// std::cout<<"camera_callback3"<<std::endl;
}

// Input : All pointcloud
// Extract bounding box point cloud
void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud)
{
	// std::cout<<"pointcloud_callback1"<<std::endl;
	ros::Time start,endd;
	
	pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
	pcl::fromROSMsg(*pointcloud, pcl_cloud);
	
	// std::cout<<"pointcloud_callback2"<<std::endl;

	// if you couldn't find the bounding box, you should publish another flags, and you try to find the new bounding box
		
	// std::cout<<"pointcloud_callback3"<<std::endl;
	std::string id = "";

	// Check all bounding box
	if(m_gBoundingboxes.bounding_boxes.size() != 0){								//core dumped problem_1
		// std::cout<<"pointcloud_callback4"<<std::endl;

		for(int i = 0; i < m_gBoundingboxes.bounding_boxes.size(); i++) {

			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//                                                      Control Input												 //
			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			double x = 0.0, y = 0.0, z = 0.0, new_z = 0.0, new_z1 = 0.0;

			// pcl::PointXYZRGB p = pcl_cloud( (m_gBoundingboxes.bounding_boxes[i].xmin + m_gBoundingboxes.bounding_boxes[i].xmax)/2,
			//                         (m_gBoundingboxes.bounding_boxes[i].ymin + m_gBoundingboxes.bounding_boxes[i].ymax)/2 );
			// x = p.x;
			// y = p.y;
			// z = p.z;
			
			double x_sum = 0.0;
			double y_sum = 0.0;
			double z_sum = 0.0;
			double cnt = 1.0;
			// std::cout<<"pointcloud_callback5"<<std::endl;
			// start =  ros::Time::now();
			// Extract bounding box point cloud
			if(!RGB_image.empty())												//core dumped problem_2
			{
				for(int ii=m_gBoundingboxes.bounding_boxes[i].ymin; ii< m_gBoundingboxes.bounding_boxes[i].ymax ; ii++)
					for(int jj=m_gBoundingboxes.bounding_boxes[i].xmin; jj< m_gBoundingboxes.bounding_boxes[i].xmax ; jj++)
					{
						
						uint8_t r = static_cast<uint8_t>(RGB_image.at<cv::Vec3b>(ii,jj)[2]), g = static_cast<uint8_t>(RGB_image.at<cv::Vec3b>(ii,jj)[1]), b = static_cast<uint8_t>(RGB_image.at<cv::Vec3b>(ii,jj)[0]);
						uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
					
						if(r <50){
							pcl::PointXYZRGB pt = pcl_cloud( jj , ii );
							pt.rgb = *reinterpret_cast<float*>(&rgb);	
							pt_data->push_back(pt);

							// Handling the Nan value
							if(!(__isnan(pt.x)) && !(__isnan(pt.y)) && !(__isnan(pt.z)) )
							{
								x_sum += pt.x;
								y_sum += pt.y;
								z_sum += pt.z;
								cnt++;
							}
						}
					}
				// std::cout<<"pointcloud_callback6"<<std::endl;
				// Get Point cloud Center x,y,z value, before clear
				x = x_sum / cnt;
				y = y_sum / cnt;
				z = z_sum / cnt;

				//Robot Coordinate		
				m_robot_pose.dis_x =  MovingAverageFilter_X(6 * (z - _distance * v2) / 7.0  + 0.291);
				m_robot_pose.dis_y =  MovingAverageFilter_Y(-1 * (x - _distance * v0) );			
				m_robot_pose.theta =  MovingAverageFilter_degree(controlInputDegree);			
				m_robot_pose.mode = 0;	
				

				if(Mode_Flag == MODE_FLAG_LP)
				{
					std::cout << "m_robot_pose.dis_x : " << m_robot_pose.dis_x << std::endl;
					std::cout << "m_robot_pose.dis_y : " << m_robot_pose.dis_y << std::endl;
					std::cout << "m_robot_pose.theta : " << m_robot_pose.theta << std::endl;

					pub_robot_pose.publish(m_robot_pose);
				}
				// Update previous robot pose
				prev_robot_pose_x = m_robot_pose.dis_x;
				prev_robot_pose_y = m_robot_pose.dis_y;
				prev_robot_pose_theta = m_robot_pose.theta;

				// TODO : if point cloud is good, outside of for moon
				
				m_gBoundingboxes.bounding_boxes.clear();
				Pub_pt.publish(pt_data);
				pt_data->clear();
			
			}
			// std::cout<<"pointcloud_callback8"<<std::endl;
			// endd =  ros::Time::now();
		}
		
	}
		// std::cout<< "Time : " << start << std::endl; 
	

}

// Input : Bounding box point cloud
// Extract one plane and segmentation
// Get normal vector
void segmentation_callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud){
	// std::cout<<"segmentation_callback1"<<std::endl;
	
	//setting variable	
	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);	
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// Convert to PCL data type
	pcl_conversions::toPCL(*pointcloud, *cloud);

	// Handling exception
	if((cloud->width==0) && (cloud->height==0)){
		return;
	}
	// std::cout<<"segmentation_callback2"<<std::endl;
	// Subsampling
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (0.03, 0.03, 0.03);
	sor.filter (cloud_filtered);

	// Copy the point cloud to change the type
	pcl::PointCloud<pcl::PointXYZ> point_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2( cloud_filtered, point_cloud);
	pcl::copyPointCloud(point_cloud, *point_cloudPtr);
	
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;

	//initialize & setting parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (point_cloudPtr);
	seg.segment (*inliers, *coefficients);
	// std::cout<<"segmentation_callback3"<<std::endl;
	// exception handling
	if (inliers->indices.size() == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		double a = 0;
		double b = 0;
		double c = 0;
		double d = 0;
		return;
	}
	
	// std::cout<<"segmentation_callback4"<<std::endl;
	// car license plate normal vector and camera normal vector -> Plane_Equation : ax + by + cz = d
	double a = coefficients->values[0];
	double b = coefficients->values[1];
	double c = coefficients->values[2];
	double d = coefficients->values[3];

	// Make a unit vector (To maintain unitvector direction forward)
	double vec_size ;

	if(d==0){d=1e-5;}

	a = -a / d;
	b = -b / d;
	c = -c / d;

	// For make size 1 vector
	vec_size = sqrt(pow(a,2) + pow(b,2) + pow(c,2));

	// Handling Exception
	if(vec_size==0){vec_size=1e-5;}

	v0 = a / vec_size;
	v1 = b / vec_size;
	v2 = c / vec_size;

	// LP normal_vec , camera z-axis
	degree = GetAngleBetweenTwoVector(v0,v2,0.0,1.0);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>);

	//std::cout << "inlier : " << inliers->indices.size() << std::endl;
	if (inliers->indices.size() == 0){
		return;
	}

	// inlier point cloud min max
	double inlierXMin = 10000.0;
	double inlierYMin = 10000.0;
	double inlierZMin = 10000.0;
	double inlierXMax = -10000.0;
	double inlierYMax = -10000.0;
	double inlierZMax = -10000.0;

	pcl::PointXYZRGB pointXMin;
	pcl::PointXYZRGB pointYMin;
	pcl::PointXYZRGB pointXMax;
	pcl::PointXYZRGB pointYMax;
	pcl::PointXYZRGB pointZMin;
	pcl::PointXYZRGB pointZMax;
	// std::cout<<"segmentation_callback5"<<std::endl;
	// inlier point cloud visualized
	// get center point of one plane
	for (std::size_t i = 0; i < inliers->indices.size (); ++i){
		for (const auto& idx: inliers->indices){
			pcl::PointXYZRGB point;
			point.x = point_cloudPtr->points[idx].x;
			point.y = point_cloudPtr->points[idx].y;
			point.z = point_cloudPtr->points[idx].z;

			point.r = 255;
			point.g = 0;
			point.b = 0;

			point_cloud_segmented->push_back(point);

			// x Min
			if(point.x < inlierXMin){
				inlierXMin = point.x;
				pointXMin = point;
			}
			// x Max
			if(point.x > inlierXMax){
				inlierXMax = point.x;
				pointXMax = point;
			}

			// y Min
			if(point.y < inlierYMin){
				inlierYMin = point.y;
				pointYMin = point;
			}
			// y Max
			if(point.y > inlierYMax){
				inlierYMax = point.y;
				pointYMax = point;
			}

			// z Min
			if(point.z < inlierZMin){
				inlierZMin = point.z;
				pointZMin = point;
			}
			// z Max
			if(point.z > inlierZMax){
				inlierZMax = point.z;
				pointZMax = point;
			}
		}
	}
	// std::cout<<"segmentation_callback6"<<std::endl;
	double pointXAvg = (inlierXMin + inlierXMax) / 2;
	double pointYAvg = (inlierYMin + inlierYMax) / 2;
	double pointZAvg = (inlierZMin + inlierZMax) / 2;
	// std::cout<<"segmentation_callback7"<<std::endl;
	// camera normal vector and car license plate center point vector
	controlInputDegree = - GetAngleBetweenTwoVector(pointXAvg, pointZAvg, 0.0, 1.0);
	// std::cout<<"segmentation_callback8"<<std::endl;
	// Visualize marker in Rviz	
	marker(v0,v1,v2,pointXAvg,pointZAvg);
	// std::cout<<"segmentation_callback9"<<std::endl;
	// Publish
	point_cloud_segmented->header.frame_id ="camera_color_optical_frame";
	Pub_pt_color.publish(point_cloud_segmented);
	point_cloud_segmented->clear();
	// std::cout<<"segmentation_callback10"<<std::endl;

}

// Input : ROI point cloud
// Find the tire
// We must change this algorithm
void segmentation_callback2(const sensor_msgs::PointCloud2ConstPtr& pointcloud){

	// create a pcl object to hold the ransac filtered results
	pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);

	//setting variable	
	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);	
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// Convert to PCL data type
	pcl_conversions::toPCL(*pointcloud, *cloud);

	// Handling exception
	if((cloud->width==0) && (cloud->height==0)){
		return;
	}

	// Subsampling
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (0.03, 0.03, 0.03);
	sor.filter (cloud_filtered);

	// Copy the point cloud to change the type
	pcl::PointCloud<pcl::PointXYZ> point_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2( cloud_filtered, point_cloud);
	pcl::copyPointCloud(point_cloud, *point_cloudPtr);
	
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;

	//initialize & setting parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (point_cloudPtr);
	seg.segment (*inliers, *coefficients);

	// // Create the filtering object
	// pcl::ExtractIndices<pcl::PointXYZRGB> extract;

	// //extract.setInputCloud (xyzCloudPtrFiltered);
	// extract.setInputCloud (point_cloudPtr);
	// extract.setIndices (inliers);
	// extract.setNegative (true);
	// extract.filter (* xyzCloudPtrRansacFiltered);
	

	// exception handling
	if (inliers->indices.size() == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		double a = 0;
		double b = 0;
		double c = 0;
		double d = 0;
		return;
	}
	
	// car license plate normal vector and camera normal vector -> Plane_Equation : ax + by + cz = d
	double a = coefficients->values[0];
	double b = coefficients->values[1];
	double c = coefficients->values[2];
	double d = coefficients->values[3];

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>);

	//std::cout << "inlier : " << inliers->indices.size() << std::endl;
	if (inliers->indices.size() == 0){
		return;
	}

	// cout << "indices : " << inliers->indices.size()<< endl;

	for (std::size_t i = 0; i < inliers->indices.size (); ++i){
		for (const auto& idx: inliers->indices){
			pcl::PointXYZRGB point;
			point.x = point_cloudPtr->points[idx].x;
			point.y = point_cloudPtr->points[idx].y;
			point.z = point_cloudPtr->points[idx].z;

			point.r = 255;
			point.g = (i * 30) % 255;
			point.b = (i * 30) % 255;

			point_cloud_segmented->push_back(point);
		}
	}

	// Publish
	point_cloud_segmented->header.frame_id ="camera_color_optical_frame";
	roi_pt__pub.publish(point_cloud_segmented);
	point_cloud_segmented->clear();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "convert_boundingbox_node");
    ros::NodeHandle nh_;

	pt_data->clear();
	pt_data->header.frame_id = "camera_color_optical_frame";

	ros::Time start;
	ros::Rate loop_rate(50);

	ros::Timer timer = nh_.createTimer(ros::Duration(0.1), timercallback1);


	image_sub = nh_.subscribe<sensor_msgs::Image>("/camera/color/image_rect_color", 1, cameraCallback);
	pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, pointcloud_callback);
	marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	marker_pub_xz = nh_.advertise<visualization_msgs::Marker>("visualization_marker_xz", 1);
	marker_pub_center = nh_.advertise<visualization_msgs::Marker>("visualization_marker_center", 1);
	pub_robot_pose = nh_.advertise<convert_2d_to_3d::robot_pose>("robot_pose",1);	


	while(ros::ok())
	{
		ros::AsyncSpinner spinner(7);
		spinner.start();

		if(Mode_Flag == MODE_FLAG_LP)
		{
			std::cout << m_targetLocationCnt << std::endl;
			bouding_box_point_cloud_sub = nh_.subscribe<sensor_msgs::PointCloud2>("/bounding_box_pointcloud", 1, segmentation_callback);		//off
			boundingboxes_sub_ = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, boundingbox_callback);	//off
			pub_result_ = nh_.advertise<convert_2d_to_3d::Result>("detected_object", 1);	// off
			Pub_pt = nh_.advertise<VPointImage>("/bounding_box_pointcloud", 10);		// off
			Pub_pt_color = nh_.advertise<VPointImage>("/colored_point_cloud", 10);		// off
		}
		if(m_targetLocationCnt >= 50){
			Mode_Flag = MODE_FLAG_STRAIGHT;
			// boundingboxes_sub_ = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, boundingbox_callback2);
			// roi_point_cloud_sub = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, segmentation_callback2);
			// roi_pt__pub = nh_.advertise<VPointImage>("/roi_segmentation_point_cloud", 10);		

		}
	}

    ros::spin();
    return 0;
}