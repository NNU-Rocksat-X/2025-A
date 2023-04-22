#!/usr/bin/env python3.7
import tensorflow as tf
import rospy
import numpy as np
import matplotlib.pyplot as plt
from mog_ai.quaternion import *
import sys
import os

gpus = tf.config.list_physical_devices('GPU')
#tf.config.experimental.set_memory_growth(gpus[0], True)
# Prevent issue where other AI models use same memory
"""
tf.config.set_logical_device_configuration(
    gpus[0],
    [tf.config.LogicalDeviceConfiguration(memory_limit=256)])
"""


print("tf version: ", tf.__version__)
print("python version: ", sys.version)

MODEL_DIR = os.getenv("TRAINING_DIR") + "grasp_vae_models/"
LOG_DIR = os.getenv("TRAINING_DIR") + "grasp_vae_logs/"

GRIPPER_POINTS = [
    [0.005959, -0.022452, 0.054853],
    [-0.023955, -0.005611, 0.053852],
    [-0.007093, 0.024302, 0.053188],
    [0.022822, 0.00746, 0.054188]]

optimizer = tf.keras.optimizers.Adam(1e-4)


def setup_model_dir(model_dir, model_name):
    
    if not os.path.exists(model_dir):
        raise Exception("Model Path does not exist!")

    highest_num = 0
    for f in os.listdir(model_dir):
        split_text = f.split(model_name+ '_')
        try:
            num = int(split_text[1][0])
            if num > highest_num:
                highest_num = num
        except IndexError:
            pass

    if highest_num == 0:
        latest_model = False
    else:
        latest_model = model_name + '_' + str(highest_num)
    next_model = model_name + '_' + str(highest_num+1)
    return (latest_model, next_model)


class GraspVae(tf.keras.Model):
    """ Variational Autoencoder that produces grasps from the latent space and 
        the ID of the object to grasp """

    def __init__(self, latent_dim, batch_size=None, model_name=None):
        super(GraspVae, self).__init__()
        self.latent_dim = latent_dim
        self.kl_divergence_weight = 1
        self.batch_size = batch_size

        self.encoder = tf.keras.Sequential(
            [
                # Input is pos_x, pos_y, pos_z, q_x, q_y, q_z, q_w
                tf.keras.layers.InputLayer(input_shape=(7,)),
                tf.keras.layers.Dense(32, activation='relu'),
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
                tf.keras.layers.Dense(32, activation='relu'),
                tf.keras.layers.Dense(7)
            ]
        )
        rospy.loginfo("model name: " + str(model_name))

        if batch_size != None:
            prev_model, self.model_shaw = setup_model_dir(MODEL_DIR, model_name)
            self.summary_writer = tf.summary.create_file_writer(LOG_DIR + self.model_shaw)

            if model_name != None and prev_model:
                print("loading model: " + MODEL_DIR + prev_model)
                self.load_weights(MODEL_DIR + prev_model)
        else:
            print("loading model: " + MODEL_DIR + model_name)
            self.load_weights(MODEL_DIR + model_name)
    # Get a grasp from the VAE
    # Either input the latent space
    # Or generate a random grasp from the latent distribution
    # Note: latent space here is referred to as epsilon in literature
    #@tf.function
    def sample(self, object_id, latent=None):
        if latent is None:
            # Concatenate the object id with the latent space
            latent = tf.random.normal(shape=(1,self.latent_dim))

        object_id = tf.reshape(object_id, shape=(latent.shape[0], 1))
        object_id = tf.cast(object_id, tf.float32)
        object_latent = tf.concat([latent, object_id], axis=1)
        
        return self.decode(object_latent), latent

    def generate_grasp(self, object_id, latent=None):
        if isinstance(latent, list):
            latent = np.array(latent)
            latent = np.reshape(latent, newshape=(1, self.latent_dim))

        grasp_out, latent = self.sample(object_id, latent)
        grasp_out = normalize_output(grasp_out)

        grasp_out = grasp_out.numpy()[0]
        grasp = Pose()
        grasp.position.x = grasp_out[0]
        grasp.position.y = grasp_out[1]
        grasp.position.z = grasp_out[2]
        grasp.orientation.x = grasp_out[3]
        grasp.orientation.y = grasp_out[4]
        grasp.orientation.z = grasp_out[5]
        grasp.orientation.w = grasp_out[6]
        return grasp
        
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

    v = tf.reshape(v, shape=(3,1))

    # Extract scalar part of quaternion
    s = q[:,3]

    # Rotates vector (essentially q*v*q_conj, but more efficient)

    vprime = 2 * tf.linalg.matmul(u,v) * u

    # Dot product of u * u
    u_dot = tf.math.multiply(u,u)
    u_dot = tf.reduce_sum(u_dot, axis=1)

    s2_part = (s*s - u_dot) * v
    s2_part = tf.transpose(s2_part)

    v = tf.reshape(v, shape=(3))
    v = tf.tile(v, [u.shape[0]])
    v = tf.reshape(v, shape=(q.shape[0], 3))
    v = tf.cast(v, dtype=tf.float64)

    cross = tf.linalg.cross(u,v)
    s_cast = tf.broadcast_to(s[:, np.newaxis], cross.shape)
    s_cross_uv = 2.0 * s_cast * cross

    out = vprime + s2_part + s_cross_uv
    return out

def normalize_quat(q):
    square = tf.math.square(q)
    sum = tf.math.reduce_sum(square, axis=1)
    norm = tf.math.sqrt(sum)
    norm = tf.broadcast_to(norm[:, np.newaxis], q.shape)
    q = q/norm
    return q


def normalize_output(logit):
    normal = logit[:,:3]
    normal = tf.concat([normal, normalize_quat(logit[:,3:])], axis=1)
    return normal

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
    #print("x: ", x)
    mean, std_dev = model.encode(x[:,1:])
    latent = model.reparameterize(mean, std_dev)
    
    object_id = tf.reshape(x[:,0], shape=(x.shape[0], 1))
    object_id = tf.cast(object_id, tf.float32)

    # Concatenate the object id with the latent space
    object_latent = tf.concat([latent, object_id], axis=1)

    x_logit = model.decode(object_latent)
    x_logit = normalize_output(x_logit)
    #print("logit: ", x_logit)

    # rl = reconstruction loss
    rl = calculate_reconstruction_loss(x, x_logit)
    rl = tf.cast(rl, tf.float32)

    t = 1 + std_dev - tf.math.square(mean) - tf.math.exp(std_dev)
    kl_loss = model.kl_divergence_weight * tf.math.reduce_mean(1 + std_dev - tf.math.square(mean) - tf.math.exp(std_dev), axis=0)
    #tf.print("rl loss: ", rl, " kl loss: ", kl_loss)

    return tf.reduce_mean(rl - kl_loss)

def visualize_latent_space(model, x):
    mean, std_dev = model.encode(x[:, 1:])
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    # Adjust c=x[:, GraspVal] to see map with different grasp attributes
    sc = ax.scatter(mean[:,0], mean[:,1], mean[:,2], c=x[:,6], cmap='brg')
    ax.set_title('Orientation W Map')
    ax.set_xlabel('dim 1')
    ax.set_ylabel('dim 2')
    ax.set_zlabel('dim 3')
    plt.colorbar(sc)
    plt.show()

#@tf.function 
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


# Set kl divergence according to Cyclical KLAnnealing schedule
# Uses linear schedule
# https://medium.com/mlearning-ai/a-must-have-training-trick-for-vae-variational-autoencoder-d28ff53b0023
def get_kl_divergence(epoch):
    anneal_rate = 5000
    if int(epoch / anneal_rate) % 2 == 0:
        kl_divergence_weight = epoch/anneal_rate - int(epoch/anneal_rate)
    else:
        kl_divergence_weight = 0.001

    return kl_divergence_weight

def train(model, dataset, epochs, validate_ratio):
    print("dataset: ", dataset.shape)
    num_validate = int(dataset.shape[0] * validate_ratio)
    validation_elements = np.random.choice(dataset.shape[0], num_validate)

    validation_set = np.array([dataset[x,:] for x in validation_elements])
    dataset = np.delete(dataset, validation_elements, axis=0)


    losses = []
    valid_loss = []
    for epoch in range(0, epochs):
        batch_elements = np.random.choice(dataset.shape[0], model.batch_size)
        batch = np.array([dataset[x,:] for x in batch_elements])

        loss = train_step(model, batch)
        validation_loss = compute_loss(model, validation_set)
        
        model.kl_divergence_weight = get_kl_divergence(epoch)
        with model.summary_writer.as_default():
            tf.summary.scalar('loss', loss, step=epoch)
            tf.summary.scalar('validation_loss', validation_loss, step=epoch)
            tf.summary.scalar('kl_divergence_weight', model.kl_divergence_weight, step=epoch)

        losses.append(loss.numpy())
        valid_loss.append(validation_loss.numpy())

        if epoch % 10 == 0:
            avg_loss = sum(losses)/len(losses)
            avg_valid_loss = sum(valid_loss)/len(valid_loss)
            print("Epoch: ", epoch, " loss: ", avg_loss, " validation loss: ", avg_valid_loss)

            # reset loss list
            losses = []
            valid_loss = []

    model.save_weights(MODEL_DIR + model.model_shaw)

if __name__ == "__main__":
    dataset = np.load("/home/cyborg/catkin_ws/src/daedalus/mog_ai/training/successful_grasps.npy")
    model = GraspVae(latent_dim=4,
                    batch_size=32, 
                    model_name="vae_4latent_4_layers_5kanneal_minibatch")

    #compute_loss(model, dataset[1:3,:])
    #train(model, dataset, 500000, 0.1)
    visualize_latent_space(model, dataset)
