import numpy as np
from sklearn.svm import SVC
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
from sklearn.preprocessing import StandardScaler
from micromlgen import port

def load_data(file_path, label):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    
    data = []
    for line in lines:
        cleaned = line.strip().replace(',', ' ').split()
        features = [float(x) for x in cleaned if x]
        if features:
            data.append(features)
    
    features = np.array(data)
    labels = np.full((features.shape[0], 1), label)
    return features, labels

# Load datasets
X1, y1 = load_data('led1_toggle.txt', 0)  # Class 0 for LED1
X2, y2 = load_data('led2_toggle.txt', 1)  # Class 1 for LED2
X3, y3 = load_data('neutral.txt', 2)      # Class 2 for Neutral

# Combine datasets
X = np.vstack((X1, X2, X3))
y = np.vstack((y1, y2, y3)).ravel()

# Calculate raw averages
avg_led1 = np.mean(X1, axis=0)
avg_led2 = np.mean(X2, axis=0)
avg_neutral = np.mean(X3, axis=0)

print("Raw averages before scaling:")
print("LED1:", ["%.4f" % x for x in avg_led1])
print("LED2:", ["%.4f" % x for x in avg_led2])
print("NEUTRAL:", ["%.4f" % x for x in avg_neutral])

# Create and fit scaler
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)

# Scale the averages to get reference vectors
scaled_led1 = scaler.transform([avg_led1])[0]
scaled_led2 = scaler.transform([avg_led2])[0]
scaled_neutral = scaler.transform([avg_neutral])[0]

print("\nReference vectors after scaling (copy to ESP32 code):")
print("const float led1_ref[] = {" + ", ".join(["%.4ff" % x for x in scaled_led1]) + "};")
print("const float led2_ref[] = {" + ", ".join(["%.4ff" % x for x in scaled_led2]) + "};")
print("const float neutral_ref[] = {" + ", ".join(["%.4ff" % x for x in scaled_neutral]) + "};")

# Split scaled data
X_train, X_test, y_train, y_test = train_test_split(X_scaled, y, test_size=0.2, random_state=42)

# Train classifier
clf = SVC(kernel='linear', gamma=0.001, class_weight={0:1, 1:1, 2:2})  # Extra weight for neutral
clf.fit(X_train, y_train)

# Evaluate
y_pred = clf.predict(X_test)
print(f"\nAccuracy: {accuracy_score(y_test, y_pred):.2f}")

# Generate C code with scaling parameters
print("\n// Copy these to your ESP32 code:")
print("// StandardScaler parameters")
print("const float scaler_mean[] = {" + ", ".join(["%.4ff" % x for x in scaler.mean_]) + "};")
print("const float scaler_scale[] = {" + ", ".join(["%.4ff" % x for x in scaler.scale_]) + "};")

# Generate classifier
c_code = port(clf, classmap={
    0: 'LED1_TOGGLE',
    1: 'LED2_TOGGLE',
    2: 'NEUTRAL'
})

with open('LEDClassifier.h', 'w') as f:
    f.write(c_code)

print("\nC code generated successfully in LEDClassifier.h")
