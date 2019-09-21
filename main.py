import tensorflow as tf
mnist = tf.keras.datasets.mnist
(x_train, y_train), (x_test, y_test) = mnist.load_data()
x_train, x_test = x_train / 255.0, x_test / 255.0

print(x_train.shape)
model = tf.keras.Sequential()
model.add(tf.keras.layers.Flatten(input_shape=(28, 28)));
model.add(tf.keras.layers.Dense(256, activation = 'relu',kernel_initializer='random_uniform'))
model.add(tf.keras.layers.Dense(128, activation = 'relu',kernel_initializer='random_uniform'))
model.add(tf.keras.layers.Dropout(0.2))
#model.add(tf.keras.layers.Dense(10, activation='softmax', input_shape=(784,)))
model.add(tf.keras.layers.Dense(10, activation = 'softmax',kernel_initializer='random_uniform'))

model.compile(loss='sparse_categorical_crossentropy',
              optimizer='adam',
              metrics=['accuracy'])

model.fit(x_train, y_train,
          batch_size=128,
          epochs=3,
          verbose=1)
#model.evaluate(x_test,y_test)

#model.save('model.h5')
# serialize model to JSON
model_json = model.to_json()
with open("model.json", "w") as json_file:
    json_file.write(model_json)
# serialize weights to HDF5
model.save_weights("model.h5")
print("Saved model to disk")

#model = tf.keras.models.load_model('model.h5')
#model.evaluate(x_test,y_test)
