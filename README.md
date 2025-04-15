# Obstacle and Pit Avoidance System

This project implements an autonomous navigation system for robots, focusing on **obstacle avoidance** and **pit detection**. The system uses **Intel RealSense D455** for point cloud data, processes this data using the **Point Cloud Library (PCL)**, and makes real-time decisions for obstacle avoidance. It also handles odometry to return to the original path after avoiding obstacles.

## 📁 Project Structure

 ├── avoidance.py # Python script for avoidance logic using odometry
 ├── pit_detection.cpp # C++ code for detecting pits
 ├── obstacle_detection.cpp # C++ code for detecting physical obstacles
 
## 🚀 Overview

This project is designed to allow a robot to autonomously navigate around obstacles and pits using the following approach:

### 1. **Obstacle Detection**:
   - **Point Cloud Generation**: The system uses the **Intel RealSense D455** camera to publish 3D point cloud data.
   - **Point Cloud Processing**: The point cloud data is processed using the **PCL (Point Cloud Library)** workspace to detect obstacles.
   - **Avoidance Logic**: Once an obstacle is detected, the robot will avoid it by altering its trajectory based on the detected obstacle's distance and position.

### 2. **Pit Detection**:
   - Similar to obstacle detection, but focuses on detecting sudden drops in terrain (like cliffs or holes), which could cause the robot to fall.

### 3. **Odometry and Path Recovery**:
   - The system uses **odometry** data to ensure that after avoiding an obstacle or pit, the robot can return to the original path, maintaining its mission objective.
