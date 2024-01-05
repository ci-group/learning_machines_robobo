# Represents a detected lane (two lines)
#
# Metadata about the frame detected
std_msgs/Header header
# Coefficients of the first line
float64[] coeffs1
# Coefficients of the second line
float64[] coeffs2
# Inverse transformation matrix, this should be used if the
# frame's perspective was transormed; else will be empty.
float64[] minv 
