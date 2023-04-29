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
    def __init__(self):
        super(DeepsatNet, self).__init__()

        # uses resnet backbone
        self.resnet = ResNet50(include_top=False, pooling='max')
        self.dense1 = tf.keras.layers.Dense(32, activation='relu')
        #self.dense2 = tf.keras.layers.Dense(32, activation='relu')
        self.dense3 = tf.keras.layers.Dense(32, activation='relu')
        #self.dense4 = tf.keras.layers.Dense(32, activation='relu')
        
        # Outputs axis angle representation (x, y, z, w) on range from (-1,1)
        self.dense5 = tf.keras.layers.Dense(4, activation='tanh')

    def call(self, x):
        x = self.resnet(x)
        x = self.dense1(x)
        #x = self.dense2(x)
        x = self.dense3(x)
        #x = self.dense4(x)
        x = self.dense5(x)
        return x

    def process_output(self, output):
        """
        Brief
            Normalizes quaternion
        Input
            Output from model, angle axis (x, y, z, w) on range (0, 1)
        Output
            Quaternion (x, y, z, w)
        """
        square = tf.math.square(output)
        square_sum = tf.math.reduce_sum(square, axis=1)
        norm = tf.math.sqrt(square_sum)

        shaped_norm = tf.repeat(norm, 4)
        shaped_norm = tf.reshape(shaped_norm, shape=(norm.shape[0], 4))

        normalized_out = tf.math.divide(output, shaped_norm)

        return normalized_out

    def compute_orientation_diff(self, q1, q2):
        """
        Brief
            Computes L2 loss two unit quaternions
            A quaternion q and -q are symmetrical so for each batch
            the loss for q and -q are calculated then the loss output 
            for that batch is the minimum loss between q and -q
        Input
            rotation1 & 2 both quaternions (x, y, z, w)
        """
        diff = q2 - q1
        l2 = tf.math.square(diff)
        loss = tf.math.reduce_mean(l2, axis=1)

        # Calculate when q2 is - because they are symmetrical
        neg_diff = -1*q2 - q1
        neg_l2 = tf.math.square(neg_diff)
        neg_loss = tf.math.reduce_mean(neg_l2, axis=1)
        
        min_loss = tf.math.minimum(loss, neg_loss)
        min_loss = tf.math.reduce_mean(min_loss)
        return min_loss



    def compute_loss(self, input, label, training=False):
        out = self.call(input)
        out = self.process_output(out)
        loss = self.compute_orientation_diff(out, label)
        
        return loss, out # return out for logging

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