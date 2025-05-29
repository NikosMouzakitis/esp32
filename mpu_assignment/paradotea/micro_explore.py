import numpy as np
import matplotlib.pyplot as plt
from sklearn.svm import SVC
from sklearn.neighbors import KNeighborsClassifier
from sklearn.ensemble import RandomForestClassifier, GradientBoostingClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.tree import DecisionTreeClassifier
from sklearn.naive_bayes import GaussianNB
from sklearn.model_selection import train_test_split, GridSearchCV
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import classification_report, accuracy_score

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
    labels = np.full((features.shape[0],), label)
    return features, labels

# Load datasets
X1, y1 = load_data('led1_toggle.txt', 0)  # Class 0 for LED1
X2, y2 = load_data('led2_toggle.txt', 1)  # Class 1 for LED2
X3, y3 = load_data('neutral.txt', 2)      # Class 2 for Neutral

# Combine datasets
X = np.vstack((X1, X2, X3))
y = np.hstack((y1, y2, y3))

# Scale data
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)

# Split into train/test
X_train, X_test, y_train, y_test = train_test_split(
    X_scaled, y, test_size=0.2, random_state=42, stratify=y
)

# Define classifiers and parameter grids
classifiers = {
    'SVM (linear kernel)': (SVC(kernel='linear', class_weight='balanced', random_state=42), {
        'C': [0.1, 1, 10, 100]
    }),
    'SVM (RBF kernel)': (SVC(kernel='rbf', class_weight='balanced', random_state=42), {
        'C': [0.1, 1, 10],
        'gamma': ['scale', 'auto', 0.01, 0.1, 1]
    }),
    'K-Nearest Neighbors': (KNeighborsClassifier(), {
        'n_neighbors': [3],  # Fixed as requested
        'weights': ['uniform', 'distance'],
        'metric': ['euclidean', 'manhattan']
    }),
    'Random Forest': (RandomForestClassifier(random_state=42), {
        'n_estimators': [50, 100, 200],
        'max_depth': [None, 5, 10],
        'min_samples_split': [2, 5],
    }),
    'Gradient Boosting': (GradientBoostingClassifier(random_state=42), {
        'n_estimators': [50, 100],
        'learning_rate': [0.01, 0.1, 0.2],
        'max_depth': [3, 5]
    }),
    'Logistic Regression': (LogisticRegression(max_iter=2000, class_weight='balanced', random_state=42), {
        'C': [0.01, 0.1, 1, 10],
        'penalty': ['l2'],
        'solver': ['lbfgs', 'saga']
    }),
    'Decision Tree': (DecisionTreeClassifier(random_state=42), {
        'max_depth': [None, 5, 10],
        'min_samples_split': [2, 5, 10]
    }),
    'Gaussian Naive Bayes': (GaussianNB(), {})  # No parameters
}

best_overall = {
    'classifier': None,
    'f1_score': 0,
    'best_params': None
}

f1_scores = []
names = []

print("Starting extensive classifier testing...\n")

for name, (clf, params) in classifiers.items():
    print(f"Testing {name}...")
    if params:
        # Grid search
        grid = GridSearchCV(clf, params, cv=5, scoring='f1_macro', n_jobs=-1)
        grid.fit(X_train, y_train)
        best_clf = grid.best_estimator_
        y_pred = best_clf.predict(X_test)
        acc = accuracy_score(y_test, y_pred)
        report = classification_report(y_test, y_pred, digits=4, output_dict=True)
        f1_macro = report['macro avg']['f1-score']
        
        print(f" Best CV params: {grid.best_params_}")
        print(f" Test accuracy: {acc:.4f}")
        print(f" Test macro F1-score: {f1_macro:.4f}")
        print(" Classification report (macro avg):")
        print(f"  Precision: {report['macro avg']['precision']:.4f}")
        print(f"  Recall:    {report['macro avg']['recall']:.4f}")
        print(f"  F1-score:  {report['macro avg']['f1-score']:.4f}")
        
        if f1_macro > best_overall['f1_score']:
            best_overall.update({
                'classifier': name,
                'f1_score': f1_macro,
                'best_params': grid.best_params_
            })
    else:
        # No parameters: train and test directly
        clf.fit(X_train, y_train)
        y_pred = clf.predict(X_test)
        acc = accuracy_score(y_test, y_pred)
        report = classification_report(y_test, y_pred, digits=4, output_dict=True)
        f1_macro = report['macro avg']['f1-score']
        
        print(f" Test accuracy: {acc:.4f}")
        print(f" Test macro F1-score: {f1_macro:.4f}")
        print(" Classification report (macro avg):")
        print(f"  Precision: {report['macro avg']['precision']:.4f}")
        print(f"  Recall:    {report['macro avg']['recall']:.4f}")
        print(f"  F1-score:  {report['macro avg']['f1-score']:.4f}")
        
        if f1_macro > best_overall['f1_score']:
            best_overall.update({
                'classifier': name,
                'f1_score': f1_macro,
                'best_params': None
            })

    f1_scores.append(f1_macro)
    names.append(name)

    print("\n" + "-"*50 + "\n")

print("=== SUMMARY ===")
print(f"Best classifier: {best_overall['classifier']}")
print(f"Best macro F1-score: {best_overall['f1_score']:.4f}")
if best_overall['best_params']:
    print(f"Best parameters: {best_overall['best_params']}")
else:
    print("No hyperparameters to tune for the best classifier.")

# Plotting the comparison of classifiers
plt.style.use('classic')

plt.figure(figsize=(10,6))
bars = plt.barh(names, f1_scores, color='skyblue')
plt.xlabel('Macro F1-Score')
plt.title('Classifier Comparison by Macro F1-Score')
plt.xlim(0, 1)

# Annotate bars with the exact F1-score
for bar in bars:
    width = bar.get_width()
    plt.text(width + 0.01, bar.get_y() + bar.get_height()/2,
             f'{width:.3f}', va='center')

plt.tight_layout()
plt.show()

