import numpy as np
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
from tensorflow.keras.utils import to_categorical
import os
import subprocess

os.environ["CUDA_VISIBLE_DEVICES"] = "-1"  # Disable all GPUs

# --- Configuration ---
NUM_SAMPLES_PER_FEATURE = 10  # your sliding window size
FEATURES_PER_SAMPLE = 2       # ay and gy per measurement
FEATURE_VECTOR_SIZE = NUM_SAMPLES_PER_FEATURE * FEATURES_PER_SAMPLE

# Map filenames to labels
data_files = {
    'led1_toggle.txt': 0,  # Class 0
    'led2_toggle.txt': 1,  # Class 1
}

# Load data
def load_data():
    X = []
    y = []
    for file, label in data_files.items():
        with open(file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split(',')
                features = [float(p) for p in parts]
                if len(features) != FEATURE_VECTOR_SIZE:
                    print(f"Warning: line in {file} has {len(features)} features, expected {FEATURE_VECTOR_SIZE}. Skipping.")
                    continue
                X.append(features)
                y.append(label)
    return np.array(X, dtype=np.float32), np.array(y)

# Compute average vector per class
def compute_class_averages(X, y):
    classes = np.unique(y)
    for cls in classes:
        samples = X[y == cls]
        avg = np.mean(samples, axis=0)
        print(f"\nAverage vector for class {cls}:")
        print(avg.tolist())  # Print as list for easier use in C later

# Build simple MLP model
def create_model(input_dim, num_classes):
    model = Sequential([
        Dense(64, activation='relu', input_shape=(input_dim,)),
        Dense(32, activation='relu'),
        Dense(num_classes, activation='softmax')
    ])
    model.compile(optimizer='adam',
                  loss='sparse_categorical_crossentropy',
                  metrics=['accuracy'])
    return model

def main():
    X, y = load_data()
    print(f"Loaded {len(X)} samples")

    compute_class_averages(X, y)

    # Shuffle dataset
    idx = np.arange(len(X))
    np.random.shuffle(idx)
    X = X[idx]
    y = y[idx]

    # Split train/test 80/20
    split = int(0.8 * len(X))
    X_train, X_test = X[:split], X[split:]
    y_train, y_test = y[:split], y[split:]

    num_classes = len(set(y))

    model = create_model(FEATURE_VECTOR_SIZE, num_classes)
    model.summary()

    model.fit(X_train, y_train, epochs=100, batch_size=16, validation_data=(X_test, y_test))

    test_loss, test_acc = model.evaluate(X_test, y_test)
    print(f"Test accuracy: {test_acc:.4f}")

    # Save TF Lite model
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    tflite_model = converter.convert()

    tflite_model_path = 'model.tflite'
    with open(tflite_model_path, 'wb') as f:
        f.write(tflite_model)
    print(f"Saved TensorFlow Lite model to {tflite_model_path}")

    # Convert .tflite to C source code array using xxd (Linux/macOS)
    c_array_file = 'model_data.cc'
    print(f"Converting {tflite_model_path} to C array in {c_array_file}...")
    subprocess.run(['xxd', '-i', tflite_model_path, c_array_file], check=True)
    print("Done!")

if __name__ == "__main__":
    main()

