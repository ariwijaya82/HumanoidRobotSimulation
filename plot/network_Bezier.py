import tensorflow as tf
from tensorflow import keras
from ann_visualizer.visualize import ann_viz;

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

train_df = pd.read_csv('./path.csv')
np.random.shuffle(train_df.values)

print(train_df.head())

model = keras.Sequential([
	keras.layers.Dense(128, input_shape=(6,), activation='softmax'),
	keras.layers.Dense(128, activation='softmax'),
	keras.layers.Dense(6, activation='softmax')])

#ann_viz(model, title="My first neural network")

model.compile(optimizer='adam', 
	          loss=keras.losses.SparseCategoricalCrossentropy(from_logits=False),
	          metrics=['accuracy'])

x = np.column_stack((train_df.righthipyaw.values, train_df.righthiproll.values, train_df.righthippitch.values, train_df.rightknee.values, train_df.rightankleroll.values, train_df.rightanklepitch.values))

# history_train = model.fit(x, train_df.time.values, batch_size=3, epochs=50)
# print(history_train.history.keys())

test_df = pd.read_csv('./servo.csv')

print(test_df.head())

test_x = np.column_stack((test_df.righthipyaw.values, test_df.righthiproll.values, test_df.righthippitch.values, test_df.rightknee.values, test_df.rightankleroll.values, test_df.rightanklepitch.values))
# print(test_x)
# print("EVALUATION")
history_test = model.fit(test_x, test_df.time.values)
# print(history_test.history.keys())

# plt.figure(1)
# plt.plot(history_train.history['accuracy'])
# plt.plot(history_train.history['loss'])
# plt.title('model training')
# plt.ylabel('train')
# plt.xlabel('time')
# plt.legend(['accuracy', 'loss'], loc='lower right')
# plt.show()

# plt.figure(2)
# plt.plot(history_test.history['accuracy'])
# plt.plot(history_test.history['loss'])
# plt.title('model training')
# plt.ylabel('test')
# plt.xlabel('time')
# plt.legend(['accuracy', 'loss'], loc='upper right')
# plt.show()