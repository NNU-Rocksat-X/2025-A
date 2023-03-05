#!/usr/bin/env python3
import tensorflow as tf
import numpy as np
from mog_ai.quaternion import *

print("tf version: ", tf.__version__)

MODEL_DIR = "../../training/grasp_vae_models/"

GRIPPER_POINTS = [
    [0.005959, -0.022452, 0.054853],
    [-0.023955, -0.005611, 0.053852],
    [-0.007093, 0.024302, 0.053188],
    [0.022822, 0.00746, 0.054188]]

KL_DIVERGENCE_WEIGHT = 0.01 # 6-DOF grasp paper used 0.01
optimizer = tf.keras.optimizers.Adam(1e-4)

class GraspVae(tf.keras.Model):
    """ Variational Autoencoder that produces grasps from the latent space and 
        the ID of the object to grasp """

    def __init__(self, latent_dim):
        super(GraspVae, self).__init__()
        self.latent_dim = latent_dim

        self.encoder = tf.keras.Sequential(
            [
                # Input is pos_x, pos_y, pos_z, q_x, q_y, q_z, q_w
                tf.keras.layers.InputLayer(input_shape=(7,)),
                tf.keras.layers.Dense(32, activation='relu'),
                # For each latent dimension two values are output:
                # mu (distribution mean) and sigma (standard deviation)
                tf.keras.layers.Dense(latent_dim + latent_dim)
            ]
        )

        self.decoder = tf.keras.Sequential(
            [  
                # Input is (Object_ID, latent_space)
                tf.keras.layers.InputLayer(input_shape=(latent_dim+1,)),
                tf.keras.layers.Dense(32, activation='relu'),
                tf.keras.layers.Dense(7,)
            ]
        )

    # Get a grasp from the VAE
    # Either input the latent space
    # Or generate a random grasp from the latent distribution
    # Note: latent space here is referred to as epsilon in literature
    @tf.function
    def sample(self, latent=None):
        if latent is None:
            # Not sure why shape=100
            latent = tf.random.normal(shape=(100, self.latent_dim))
        return self.decode(latent)
        
    def encode(self, x):
        # split the mean and std_dev from the encoder output
        # mean and std_dev are vectors of size latent_dim
        mean, std_dev = tf.split(self.encoder(x), num_or_size_splits=2, axis=1)
        return mean, std_dev

    # Uses the mean and std_dev generated from the encoder
    # to generate the latent vector
    # tf.exp(std_dev * 0.5) is just the variance
    def reparameterize(self, mean, std_dev):
        eps = tf.random.normal(shape=mean.shape)
        return mean + tf.exp(std_dev * 0.5) * eps

    def decode(self, latent):
        logits = self.decoder(latent)
        return logits

# v and q are lists where
# v = [x, y, z]
# q = [x, y, z, w]
# https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
def rotate_vect_by_quat(v, q):
    # Extract vector part of quaternion
    u = q[:,:-1]

    v = tf.tile(v, [u.shape[0]])
    v = tf.reshape(v, shape=(u.shape[0], 3))
    v = tf.cast(v, dtype=tf.float64)

    # Extract scalar part of quaternion
    s = q[:,3]

    # Rotates vector (essentially q*v*q_conj, but more efficient)
    # tensor dot performs dot product on a batch
    # linalg.cros is cross product on batch
    vprime = 2.0 * tf.tensordot(u, v,2) * u
    s2_part = s*s - tf.tensordot(u,u,2)
    s2_cast = tf.broadcast_to(s2_part[:, np.newaxis], v.shape)
    vprime = vprime + s2_cast * v
    cross = tf.linalg.cross(u,v)
    s_cast = tf.broadcast_to(s[:, np.newaxis], cross.shape)
    vprime = vprime + 2.0 * s_cast * tf.linalg.cross(u, v)

    return vprime

# grasp = [pos_x, pos_y, pos_z, q_x, q_y, q_z, q_w]
def grasp_to_gripper_points(grasp):
    transformed_points = []
    i = 0
    grasp = tf.cast(grasp, tf.float64)
    for point in GRIPPER_POINTS:
        rotated_point = rotate_vect_by_quat(point, grasp[:,3:])
        translated = tf.add(rotated_point, grasp[:,:3])
        transformed_points.append(translated)
        i+=1
    return tf.concat(transformed_points, axis=0)

def calculate_reconstruction_loss(x_grasp, logit_grasp):
    x_points = grasp_to_gripper_points(x_grasp[:,1:])
    logit_points = grasp_to_gripper_points(logit_grasp)

    error = tf.math.abs(x_points - logit_points)

    # Calculates mean between each element in x_grasp and logit_grasp
    # return array of x_grasp.shape and logit_grasp.shape (they are equal)
    # error_mean[0,0] = (x_grasp[0,0] + logit_grasp[0,0]) / 2
    error_mean = tf.math.reduce_mean(error, axis=0)

    # Add up the error for each axis for each point
    # returns single error value
    error_mean = tf.math.reduce_sum(error_mean)
    return error_mean



def compute_loss(model, x):
    # Encode the grasp, but do not encode the object id
    mean, std_dev = model.encode(x[:,1:])
    latent = model.reparameterize(mean, std_dev)
    
    object_id = tf.reshape(x[:,0], shape=(x.shape[0], 1))
    object_id = tf.cast(object_id, tf.float32)

    # Concatenate the object id with the latent space
    object_latent = tf.concat([latent, object_id], axis=1)

    x_logit = model.decode(object_latent)

    # rl = reconstruction loss
    rl = calculate_reconstruction_loss(x, x_logit)
    rl = tf.cast(rl, tf.float32)

    t = 1 + std_dev - tf.math.square(mean) - tf.math.exp(std_dev)
    kl_loss = KL_DIVERGENCE_WEIGHT * tf.math.reduce_mean(1 + std_dev - tf.math.square(mean) - tf.math.exp(std_dev), axis=0)
    #tf.print("rl loss: ", rl, " kl loss: ", kl_loss)

    return tf.reduce_mean(rl - kl_loss)

@tf.function 
def train_step(model, x):
    """Executes one training step and returns the loss.

    This function computes the loss and gradients, and uses the latter to
    update the model's parameters.
    """
    with tf.GradientTape() as tape:
        loss = compute_loss(model, x)
    gradients = tape.gradient(loss, model.trainable_variables)
    optimizer.apply_gradients(zip(gradients, model.trainable_variables))
    return loss # for logging

def train(epochs, validate_ratio):
    dataset = np.load("/home/cyborg/catkin_ws/src/daedalus/mog_ai/training/successful_grasps.npy")
    print("dataset: ", dataset.shape)

    num_validate = int(dataset.shape[0] * validate_ratio)
    validation_elements = np.random.choice(dataset.shape[0], num_validate)

    validation_set = np.array([dataset[x,:] for x in validation_elements])
    dataset = np.delete(dataset, validation_elements, axis=0)

    model = GraspVae(latent_dim=3)
    model.load_weights(MODEL_DIR + "model_1")


    losses = []
    valid_loss = []
    for epoch in range(0, epochs):
        loss = train_step(model, dataset)
        validation_loss = compute_loss(model, validation_set)

        losses.append(loss.numpy())
        valid_loss.append(validation_loss.numpy())

        if epoch % 10 == 0:
            avg_loss = sum(losses)/len(losses)
            avg_valid_loss = sum(valid_loss)/len(valid_loss)
            print("Epoch: ", epoch, " loss: ", avg_loss, " validation loss: ", avg_valid_loss)

            # reset loss list
            losses = []
            valid_loss = []

    model.save_weights(MODEL_DIR + "model_1")

if __name__ == "__main__":
    train(10000, 0.1)

