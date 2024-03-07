#! /bin/bash

echo "Move the all gazebo models into ~/.gazebo/models directory." 
cp -R ../../gazebo_apriltag/models/* ~/.gazebo/models/
echo '...'
echo "Remove the gazebo_apriltag directory"
rm -rf ../../gazebo_apriltag
echo '...'
echo "End..!"
