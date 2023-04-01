import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import Pose, Point, Twist
from mog_ai.quaternion import rotate, normalize_quat

def npPoint(np_arr):
    return Point(np_arr[0], np_arr[1], np_arr[2])

def shift_point_to_frame(point: Point, frame: Pose) -> Pose:
    """
    Moves frame so the point within the frame is at the original origin
    """
    point_arr = [point.x, point.y, point.z]
    rotated_point = rotate(point_arr, frame.orientation)

    frame_arr = np.array([frame.position.x, frame.position.y, frame.position.z])

    translated_frame = frame_arr - rotated_point

    # Place result back in frame
    frame.position.x = translated_frame[0]
    frame.position.y = translated_frame[1]
    frame.position.z = translated_frame[2]
    return frame


def set_frame(frame: Pose, frame2: Point):
    """
    Translate second frame to the inertial frame of the first frame
    """
    translated = Point(frame.position.x + frame2.x,
                       frame.position.y + frame2.y,
                       frame.position.z + frame2.z)
    return translated

def place_point_on_vector(point, vector):
    """ 
    Moves point to the closest point on a vector

    Input
    :param point: 3d numpy array
    :param vector: 3d numpy array

    Output
    :return translated point as 3d numpy array
    """
    # Calculate the vector connecting the point and the vector's base point
    vector_to_point = point - vector[0]
    # Calculate the normal vector of the plane perpendicular to the vector
    normal_vector = np.cross(vector[1]-vector[0], np.cross(vector_to_point, vector[1]-vector[0]))
    # Calculate the projection of the vector connecting the point and the vector's base point onto the plane perpendicular to the vector
    projection = np.dot(vector_to_point, normal_vector) / np.dot(normal_vector, normal_vector) * normal_vector
    # Translate the point to the closest point on the vector
    translated_point = vector[0] + vector_to_point - projection
    return translated_point

def time_to_reach_position(pose_0: Pose, pose_1: Pose, velocity: Twist) -> float:
    """
    Calculates the time it takes for an object starting at position_0 to reach position_1
    when the velocity vector is given.

    Args:
        position_0 (geometry_msgs.msg.Point): Starting position of the object.
        position_1 (geometry_msgs.msg.Point): Final position of the object.
        velocity (geometry_msgs.msg.Vector3): Velocity vector of the object.

    Returns:
        float: The time it takes for the object to reach position_1 in seconds.
    """
    position_0 = pose_0.position
    position_1 = pose_1.position 

    position_0_array = np.array([position_0.x, position_0.y, position_0.z])
    position_1_array = np.array([position_1.x, position_1.y, position_1.z])
    velocity_array = np.array([velocity.linear.x, velocity.linear.y, velocity.linear.z])

    displacement = position_1_array - position_0_array
    distance = np.linalg.norm(displacement)
    speed = np.linalg.norm(velocity_array)
    time = distance / speed

    return time

#############################################################
###                     TESTING
#############################################################
def plot_place_point_on_vector():
    # Define the point and vector
    point = np.array([1, 2, 3])
    vector = np.array([[0, 0, 0], [4, 4, 4]])

    # Translate the point to the vector
    translated_point = place_point_on_vector(point, vector)

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the original point and vector
    ax.scatter(point[0], point[1], point[2], c='r', marker='o', label='Original point')
    ax.quiver(vector[0, 0], vector[0, 1], vector[0, 2], vector[1, 0]-vector[0, 0], vector[1, 1]-vector[0, 1], vector[1, 2]-vector[0, 2], color='b', label='Vector')

    # Plot the translated point
    ax.scatter(translated_point[0], translated_point[1], translated_point[2], c='g', marker='o', label='Translated point')
    ax.plot([point[0], translated_point[0]], [point[1], translated_point[1]], [point[2], translated_point[2]], color='m', linestyle='--', label='Projection')

    # Set the axis labels and legend
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    # Show the plot
    plt.show()

def test_shift_point_to_frame():
    point = Point([4, 5, 6])
    pose = Pose()
    pose.position = Point(1, 2, 3)
    pose.orientation = normalize_quat(pose.orientation)
    print("Point: ", point)
    print("frame: ", pose)

    pose = shift_point_to_frame(point, pose)
    print("shifted: ", pose)


if __name__ == "__main__":
    test_shift_point_to_frame()
