# convert_2d_to_3d

This node publishs the center 3D points position of the 2D detected boxes.  
Detected boxes are from darknet_ros package.  

Type of subscribes : [sensor_msgs::PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html/) , [darknet_ros_msgs::BoundingBoxes](https://github.com/leggedrobotics/darknet_ros/blob/master/darknet_ros_msgs/msg/BoundingBox.msg)  
Type of publish : [convert_2d_to_3d::Result](https://github.com/oorrppp2/convert_2d_to_3d/blob/master/msg/Result.msg)  

## How to run
<pre>
<code>
rosrun convert_2d_to_3d convert_boundingbox_node
</code>
</pre>
