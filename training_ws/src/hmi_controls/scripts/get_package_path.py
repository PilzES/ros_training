#!/usr/bin/env python
import rospy
import rospkg
import sys

def get_path_to_item(relative_filename):

    package_name = "hmi_controls"
    rospack = rospkg.RosPack()
    current_package = rospack.get_path(package_name)
    file_path = current_package + "/" + relative_filename
    return file_path

if __name__ == "__main__":

    # Create a rosnode
    rospy.init_node('package_path_finder')

    folder = "config/hmi_interface"

    folder_path = get_path_to_item(folder)
    print(folder_path)
    sys.exit(0)