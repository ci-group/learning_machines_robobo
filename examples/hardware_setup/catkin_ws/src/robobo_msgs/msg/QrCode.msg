# Represents a detected QR Code
#
# Metadata about the frame
Header header
# Text contained on the QR Code
string text
# Distance between the center of the code and
# the first result point
float32 distance
# Coordinate of the center of the code
opencv_apps/Point2D center
# Vector of coordinates of the result points
opencv_apps/Point2D[] result_points 
