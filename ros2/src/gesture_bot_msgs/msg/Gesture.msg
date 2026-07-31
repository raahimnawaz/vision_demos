# One frame of hand perception.
#
# Mirrors gesture_bot.perception.GestureObservation. There is no standard ROS
# message for canned hand gestures, so this is the one custom interface in the
# stack -- detections use vision_msgs and motion uses geometry_msgs/Twist.

std_msgs/Header header

# Recognized gesture label, e.g. "Open_Palm". Empty string when no gesture was
# recognized (including when a hand is present but its pose matches nothing).
string gesture

# Gesture confidence in [0, 1]. Zero when gesture is empty.
float32 score

# False when no hand is in frame at all. The decision layer treats a run of
# these as a dead-man and stops the robot.
bool hand_present

# Hand centroid, normalized to [0, 1] in DISPLAY coordinates -- the frame as the
# user sees it, i.e. horizontally mirrored relative to a raw webcam feed.
# x = 0 is image-left, y = 0 is image-top.
#
# Publishers must mirror before measuring (perception.to_display_frame, applied
# by GestureSource.prepare). Publishing raw camera coordinates here inverts the
# sign of decision.py's steering term rather than merely offsetting it, so the
# robot trims the wrong way while every unit test still passes.
float32 hand_x
float32 hand_y

# "Left" / "Right", or empty when unknown.
string handedness
