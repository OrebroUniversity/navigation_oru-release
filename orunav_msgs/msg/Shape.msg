# Defines a shape message
# A shape can be either a Circle or a Polygon
# For a Circle, the first point represents the centre and 
# the x on the second represents the radius
# For a Polygon, all the points are points in the polygon

int32 CIRCLE = 0
int32 POLYGON = 1

int32 type
geometry_msgs/Point[] points 