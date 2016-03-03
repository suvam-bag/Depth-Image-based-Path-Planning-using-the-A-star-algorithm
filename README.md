# Depth-Image-based-Path-Planning-using-the-A-star-algorithm
Created by Suvam Bag in the Multi-Agent-Biorobotics-Laboratory in Rochester Institute of Technology.

#Introduction
Path planning is one of the most important branches of autonomous cars both in indoor and outdoor environments. The A* algorithm is a heuristic search algoorithm capable of finding the most optimized path to a destination provided a map. The map was built using the depth image of a Asus Xtion sensor and the robot used is an AmigoBot from Adept Mobile Robots. 

  1. Operating System - Ubuntu 14.04
  2. Frameworks - ROS Indigo
  3. Libraries - OpenCV, OpenNI, RosAria
  4. Hardware - AmigoBot, Asus Xtion
  5. Language - Python

#Description
A* is a very famous search algorithm described Peter Hart, Nils Nilsson and Bertram Raphael of Stanford Research Institute (now SRI International) in 1968. It is a modified version of Dijkstra's algorithm with a heuristic function added to it. There are many options of the heuristic fucntion, the one used in this project is the manhattan distance between the starting position and the destination. The map used is a binary 2D grid map formed from the depth image of the Xtion. The depth image was converted to a 2D array before transfering to the path planning section of algorithm. The codes for the path planning and the depth image processing have been uploaded in two separate parts. The paper written for the project has also been uploaded for a more detailed description. 

#Installation Links

  1. ROS Indigo on Ubuntu 14.04 - http://wiki.ros.org/indigo/Installation/Ubuntu
  2. OpenCV - http://www.samontab.com/web/2014/06/installing-opencv-2-4-9-in-ubuntu-14-04-lts/
  3. OpenNI2 - http://samouresearch.blogspot.com/2015/03/install-openni2-on-ubuntu-1404-with.html

#Link to Paper
  https://drive.google.com/open?id=0B5BTnIxc0GZzeFdwUi1rTUdlRjA
