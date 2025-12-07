# Computer Vision and Depth Perception Integration

This project demonstrates integration of computer vision and depth perception using OpenCV, PCL concepts, and ROS 2 for robotic navigation.

## Components

- `opencv_ros_integration.py`: Example of integrating OpenCV with ROS 2 for 2D image processing
- `pcl_ros_integration.py`: Conceptual example of PCL integration with ROS 2 for 3D point cloud processing
- `depth_perception_demo.py`: Example of processing depth camera data for 3D information extraction
- `vision_navigation_agent.py`: Complete agent that uses both 2D and 3D vision for navigation
- `README.md`: This documentation file

## How it Works

The vision navigation agent integrates multiple perception modalities:

1. **2D Vision Processing**: Uses OpenCV to process RGB images for obstacle detection
2. **3D Depth Processing**: Processes depth images and point clouds for spatial understanding
3. **Sensor Fusion**: Combines 2D and 3D information for robust obstacle detection
4. **Navigation**: Uses vision data to navigate and avoid obstacles

## Running the Examples

1. Make sure you have ROS 2, OpenCV, and sensor message packages installed
2. Launch a robot simulation with camera and depth sensors
3. Run the OpenCV integration example:
   ```bash
   python3 opencv_ros_integration.py
   ```

4. Run the depth perception demo:
   ```bash
   python3 depth_perception_demo.py
   ```

5. Run the complete vision navigation agent:
   ```bash
   python3 vision_navigation_agent.py
   ```

## Key Concepts Demonstrated

- **cv_bridge**: Converting between ROS image messages and OpenCV formats
- **Depth processing**: Converting depth images to 3D point clouds
- **Point cloud filtering**: Basic filtering and analysis of 3D data
- **Sensor fusion**: Combining 2D and 3D perception for navigation
- **Real-time processing**: Efficient image and point cloud processing for robotics

## Expected Behavior

The vision navigation agent will:
- Process RGB and depth images in real-time
- Detect obstacles using both 2D and 3D information
- Navigate safely around obstacles
- Log perception results and navigation decisions