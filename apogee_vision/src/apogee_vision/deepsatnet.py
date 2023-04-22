import tensorflow as tf
import numpy as np
import math
from tensorflow.keras.applications.resnet50 import ResNet50
from tensorflow.keras.applications.resnet import preprocess_input
from mog_ai import quaternion
from mog_ai.grasp_vae import rotate_vect_by_quat
import copy

gpus = tf.config.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(gpus[0], True)

#optimizer = tf.keras.optimizers.Adam(1e-4)
optimizer = tf.keras.optimizers.experimental.SGD(learning_rate=1e-4)
DEBUG = False

def q_tf_conj(q):
    q_conj = copy.deepcopy(q)
    q_conj[:, 0] = -1 * q[:, 0]
    q_conj[:, 1] = -1 * q[:, 1]
    q_conj[:, 2] = -1 * q[:, 2]
    return q_conj

def rotate_tensor(v, q):
    #print("v", v)
    qv = tf.concat([v, [0]], 0)
    qv = tf.tile(qv, [q.shape[0]])
    qv = tf.reshape(qv, shape=(q.shape[0], 4))
    #print("qv:", qv)

    # quat conjugate
    #print("q:", q)
    q_conj = q_tf_conj(q)
       
    #print("q conj", q_conj)
    #print("q after", q)

    q_qv = quaternion.quaternion_multiply(q[0], qv[0].numpy())
    #print("q_qv:", q_qv)

def tf_rotate(v, q):
    vw = tf.zeros((1), dtype=tf.dtypes.float32)
    qv = tf.concat([v, vw], 0)
    #print("qv:", qv)

def transform_points(q):
    points = tf.constant([[-1, -1, -1],
                        [1, -1, -1],
                        [1, 1, -1],
                        [-1, 1, -1],
                        [-1, -1, 1],
                        [1, -1, 1],
                        [1, 1, 1],
                        [-1, 1, 1]], dtype=tf.dtypes.float64)

    q = tf.cast(q, dtype=tf.float64)

    trans_points = []
    for point in points:
        trans_point = rotate_vect_by_quat(point, q)
        trans_points.append(trans_point)
    trans_points = tf.concat(trans_points, axis=0)
    trans_points = tf.reshape(trans_points, (q.shape[0], points.shape[0], points.shape[1]))
    return trans_points

def angle_axis_to_quat(aa):
    """
    Input
        numpy angle axis (x, y, z, w)
    Output
        numpy quaternion (x, y, z, w)
    """
    q = tf.concat([
        aa[:, 0] * tf.math.sin(aa[:, 3] / 2),
        aa[:, 1] * tf.math.sin(aa[:, 3] / 2),
        aa[:, 2] * tf.math.sin(aa[:, 3] / 2),
        tf.math.cos(aa[:, 3] / 2)
    ], 0)
    q = tf.reshape(q, aa.shape)
    return q

def closest_point_error(p1, p2):
    """
    Brief
        Takes two sets of points, for each point calculates the distance to the closest point
        This is avgeraged to result in an error

    Input
        p1 and p2 are tf.tensors (batch, num_points, 3)
    """
    closest_point_errors = []
    for p1_batch, p2_batch in zip(p1, p2):
        for point in p1_batch:
            # Calculate distance between point from p1 and each point in p2
            diff = tf.math.subtract(point, p2_batch)
            diff_2 = tf.square(diff)
            sum_diff_2 = tf.reduce_sum(diff_2, axis=1)
            dist = tf.sqrt(sum_diff_2)
            closest_point_error = tf.math.reduce_min(dist)
            closest_point_errors.append(closest_point_error)

    closest_point_errors = tf.stack(closest_point_errors, 0)
    #print("closest_point errors", closest_point_errors)

    # Reshape to (batches, points)
    closest_point_errors = tf.reshape(closest_point_errors, (p1.shape[0], p1.shape[1]))
    closest_point_errors = tf.reduce_mean(closest_point_errors, axis=1)
    #print("reduced closest_point errors", closest_point_errors)
    return closest_point_errors



class DeepsatNet(tf.keras.Model):
    def __init__(self, icp_weight, quat_divergence_weight):
        super(DeepsatNet, self).__init__()

        self.quat_divergence_weight = tf.constant(quat_divergence_weight, dtype=tf.float32)
        self.icp_weight = tf.constant(icp_weight, dtype=tf.float64)

        # uses resnet backbone
        self.resnet = ResNet50(include_top=False, pooling='max')
        #self.dense1 = tf.keras.layers.Dense(1024, activation='relu')
        #self.dense2 = tf.keras.layers.Dense(32, activation='relu')
        self.dense3 = tf.keras.layers.Dense(32, activation='relu')
        #self.dense4 = tf.keras.layers.Dense(32, activation='relu')
        
        # Outputs axis angle representation (x, y, z, w)
        self.dense5 = tf.keras.layers.Dense(4, activation='sigmoid')

    def call(self, x):
        x = self.resnet(x)
        #x = self.dense1(x)
        #x = self.dense2(x)
        x = self.dense3(x)
        #x = self.dense4(x)
        x = self.dense5(x)
        return x

    def process_output(self, output):
        """
        Brief
            Transform axis angle on range (0, 1) to (0, pi/2) then 
            convert to quaternion
            Output may still be ambiguous due to symmetries, 
            this will be resolved later by comparing to previous orientation
        Input
            Output from model, angle axis (x, y, z, w) on range (0, 1)
        Output
            Quaternion (x, y, z, w)
        """
        #print("output:", output)
        normalized_angle_axis = output * math.pi/2
        #print("norm_axis", normalized_angle_axis)

        #print("norm out:", normalized_angle_axis)
        q_out = angle_axis_to_quat(normalized_angle_axis)
        #print("q out:", q_out)
        return q_out

    def compute_orientation_diff(self, q1, q2):
        """
        Brief
            Transforms points with rotations and calculates error between closest points
            Closest points accounts for symmetries
        Input
            rotation1 & 2 both quaternions (x, y, z, w)
        """
        q2_points = transform_points(q2)
        q1_points = transform_points(q1)
        if DEBUG:
            print("label:", q2)
            print("label points:", q2_points)
            print("out:", q1)
            print("out_points:", q1_points)

        error = closest_point_error(q1_points, q2_points)
        #print("error", error)

        # sums batch errors
        error = tf.math.reduce_sum(error)
        #print("reduced err", error)
        return error


    # Use distance for now
    # label is np array (batch, labels) where labels = (x, y, z, w) of quaternion
    def compute_loss(self, input, label, training=False):
        out = self.call(input)
        #print("out:", out)
        out = self.process_output(out)
        error = self.compute_orientation_diff(out, label)

        # Quat loss adds error proportional to difference between quaternion label and output
        # Prevents model from learning single value close enough to all cases (local min)
        
        if training:
            quat_error = tf.math.abs(out - label)
            quat_error = tf.math.reduce_mean(quat_error, axis=0)
            quat_error = tf.math.reduce_mean(quat_error)
            quat_loss = quat_error * self.quat_divergence_weight
            quat_loss = tf.cast(quat_loss, dtype=tf.float64)

            error = error * self.icp_weight
        else:
            quat_loss = 0
        

        error = error + quat_loss
        return error, out

    #@tf.function
    def train_step(self, input, label):
        with tf.GradientTape() as tape:
            loss, out = self.compute_loss(input, label, training=True)
        gradients = tape.gradient(loss, self.trainable_variables)
        optimizer.apply_gradients(zip(gradients, self.trainable_variables))
        return loss, gradients # for logging

    def inference(self, input):
        out = self.call(input)
        out = self.process_output(out)
        return out

if __name__ == "__main__":
    # Size of 224 needed for imagenet
    image = np.ones(shape=(1, 224,224,3))

    net = DeepsatNet()

    out = net(image)

    print(out.shape)