
def parse_state(state):
    gps = state.gps_location
    kinematic = state.kinematics_estimated
    time_stamp = state.timestamp
    return gps, kinematic, time_stamp


def is_new_collision(prev_collision, new_collision):
    return not(
            prev_collision.object_name == new_collision.object_name and
            compare_vectors(prev_collision.impact_point, new_collision.impact_point) and
            compare_vectors(prev_collision.normal, new_collision.normal) and
            compare_vectors(prev_collision.position, new_collision.position)
    )


def compare_vectors(vec1, vec2, thr=0.001):
    return abs(vec1.x_val - vec2.x_val) <= thr \
           and abs(vec1.y_val - vec2.y_val) <= thr\
           and abs(vec1.z_val - vec2.z_val) <= thr


def get_position(kinematic):
    pos = kinematic.position
    return pos.x_val, pos.y_val, pos.z_val


def get_velocity(kinematic):
    vel = kinematic.linear_velocity
    return vel.x_val, vel.y_val, vel.z_val


def get_gps_info(gps_val):
    return gps_val.altitude, gps_val.latitude, gps_val.longitude
