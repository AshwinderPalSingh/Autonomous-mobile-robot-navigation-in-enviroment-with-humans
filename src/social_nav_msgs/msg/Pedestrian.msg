# One detected (or simulated) pedestrian.
# position and velocity are expressed in the frame given by the
# parent Pedestrians message header.

string id                        # stable track id, e.g. "ped_1"
geometry_msgs/Point position     # [m]
geometry_msgs/Vector3 velocity   # [m/s], world-aligned in header frame
