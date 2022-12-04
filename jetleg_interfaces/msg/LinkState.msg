# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each link is defined by:
#  * the position of the link (XYZ),
#  * the orientation of the link (XYZW) and 
#
# Each link is uniquely identified by its name
# The header specifies the time at which the link states were recorded. All the link states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the link state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the link name with the correct
# states.


std_msgs/Header header

string[] name
geometry_msgs/Point[] position
geometry_msgs/Quaternion[] orientation
