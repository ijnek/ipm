# This represents a Point2D with reference coordinate frame and timestamp

std_msgs/Header header  # Header timestamp should be acquisition time of image
                        # Header frame_id IS IGNORED in Inverse Perspective Mapping, because it should be in image coordinates.
vision_msgs/Point2D point
