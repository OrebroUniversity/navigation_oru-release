#!/bin/bash
echo "Parsing bag files in path: $1"

for f in $1*.bag
do
  echo "Processing $f file..."
  # take action on each file. $f store current file name
  #echo "rosbag filter $f $f.filtered.bag"
  rosbag filter $f $f.filtered.bag "topic == '/tf' or topic == '/laserscan'"
done
