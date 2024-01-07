# Represents a detected Color Blob
#
# Metadata about the frame
Header header
# Color of the blob
std_msgs/String color
# Size of the blob (area) in pixels
std_msgs/Int32 size
# Coordinate of the center of the blob
opencv_apps/Point2D center

