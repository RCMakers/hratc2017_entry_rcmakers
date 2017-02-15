This package is team RCMakers's submission for the HRATC 2017.

## Dependencies
### Utilities
`pip`
`python-rosdep`

### Python Libraries
`python-numpy`  
`scikit-learn (v0.18.x)` (not available on APT)  

### ROS Packages
`metal_detector_msgs`  
`roscpp`  
`rospy`  
`tf`  
`std_msgs`  
`message_filters`  
`move_base_msgs`  
`actionlib`  
`angles`  
`laser_assembler`  
`control_msgs`  
`gmapping`  
`openslam_gmapping`  
`map_server`  
`robot_localization`  

## Installation
First, copy the package folder (rename it to `hratc2017_entry_rcmakers` if its name is different) to the `src` folder in the `hratc2017_workspace`.  

### Installing Dependencies
In an account with sudo privileges, navigate to the root directory of the workspace and run the following:  
`$ rosdep install --from-paths src --ignore-src --rosdistro=indigo`  
You will be prompted for your password to perform the necessary APT operations.  
Following this, first uninstall any current instances of `python-sklearn` you may have on your computer, as they will conflict with the latest version of `scikit-learn`. To do this and install `scikit-learn`, use the following commands:  
`$ sudo apt-get remove python-sklearn`  
`$ sudo pip install -U scikit-learn`  

### Finalization
In the root directory of your workspace, run the following:  
`$ catkin_make`.  

## Usage
With the navigation stack, sensors, and simulation (if applicable) running, run the following command:  
`$ roslaunch hratc2017_entry_rcmakers hratc2017_rcmakers.launch`

## Contributions
* Ege Çağlar: Navigation and localization
* Tan Gemicioğlu: Mine detection, methods research
* Ali Çataltepe: System design and debugging