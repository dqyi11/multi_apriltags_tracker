#include <ros/ros.h>
#include "multi_apriltags_tracker/multi_april_tags_tracker.h"

int main( int argc, char** argv ) {
  ros::init( argc, argv, "multi_apriltags_tracker" );
  MultiAprilTagsTracker tracker;  
  ros::spin();
  return 0;
}
