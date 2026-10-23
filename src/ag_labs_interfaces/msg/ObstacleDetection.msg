# ObstacleDetection.msg
# Represents detected obstacles with polygon vertices in the robot's source frame.
# Vertices are sorted counter-clockwise.

std_msgs/Header header
geometry_msgs/Polygon[] obstacles
