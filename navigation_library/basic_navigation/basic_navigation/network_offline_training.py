import pandas as pd
import tensorflow as tf
import os
from geometry_msgs.msg import Twist
import numpy as np
import math


class OfflineTrainer:
    def __init__(self):
        self.nn = tf.keras.models.Sequential([
            tf.keras.layers.Dense(10, activation='relu', input_shape=(4,)),
            tf.keras.layers.Dense(2)
        ])
        self.nn.compile(optimizer='adam', loss='mean_squared_error')

    def train(self):
        # Load the data from the CSV file
        data = pd.read_csv(os.path.join('/home/andrew/repo/dokalman/navigation_library/basic_navigation/tf_models', 'data.csv'))

        # Calculate the sine and cosine of the bearing and heading
        data['bearing_sin'] = np.sin(np.deg2rad(data['bearing']))
        data['bearing_cos'] = np.cos(np.deg2rad(data['bearing']))
        data['heading_sin'] = np.sin(np.deg2rad(data['heading']))
        data['heading_cos'] = np.cos(np.deg2rad(data['heading']))

        # Remove the first and last 100 rows
        data = data[100:-100]

        # Split the data into inputs (bearing_sin, bearing_cos, heading_sin, heading_cos) and outputs (angular_z, linear_x)
        inputs = data[['bearing_sin', 'bearing_cos', 'heading_sin', 'heading_cos']].values
        outputs = data[['angular_z', 'linear_x']].values

        # Oversample data where angular_z is not 0
        non_zero_indices = np.where(outputs[:, 0] != 0)[0]
        inputs = np.concatenate([inputs, np.repeat(inputs[non_zero_indices], 9, axis=0)])
        outputs = np.concatenate([outputs, np.repeat(outputs[non_zero_indices], 9, axis=0)])

        # Train the neural network
        self.nn.fit(inputs, outputs, epochs=250)

        

    def save_weights(self):
        # Save the weights of the neural network
        self.nn.save_weights(os.path.join('/home/andrew/repo/dokalman/navigation_library/basic_navigation/tf_models', 'weights.h5'))


def main():
    trainer = OfflineTrainer()
    trainer.train()
    trainer.save_weights()

if __name__ == '__main__':
    main()