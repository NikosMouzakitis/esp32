import numpy as np
from sklearn.svm import SVC
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
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

# Combine datasets
X = np.vstack((X1, X2))
y = np.vstack((y1, y2)).ravel()

# Split data
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Create and train the classifier with explicit gamma
clf = SVC(kernel='linear', gamma=0.001)  # Added gamma parameter
clf.fit(X_train, y_train)

# Evaluate
y_pred = clf.predict(X_test)
print(f"Accuracy: {accuracy_score(y_test, y_pred):.2f}")

# Generate C code
c_code = port(clf, classmap={
    0: 'LED1_TOGGLE',
    1: 'LED2_TOGGLE'
})

# Save to header file
with open('LEDClassifier.h', 'w') as f:
    f.write(c_code)

print("C code generated successfully in LEDClassifier.h")
