# This message contains an array of events
# (0, 0) is at the top left corner of image

std_msgs/Header header
uint32 height           # image height, that is, number of rows
uint32 width            # image width, that is, number of columns
Event[] events          # an array of events