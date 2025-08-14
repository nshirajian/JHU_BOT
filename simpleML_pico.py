import pandas as pd
import numpy as np
import os
from sklearn.tree import DecisionTreeClassifier, export_text
from sklearn.metrics import confusion_matrix, classification_report

BASE_DIR = "Training Data"
FEATURES = ["acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"]

dfs = []
for folder in sorted(os.listdir(BASE_DIR)):
    folder_path = os.path.join(BASE_DIR, folder)
    if not os.path.isdir(folder_path):
        continue
    label = folder.lower()
    for file in os.listdir(folder_path):
        if file.endswith(".csv"):
            df = pd.read_csv(os.path.join(folder_path, file))
            df["label"] = label
            dfs.append(df)

df_all = pd.concat(dfs, ignore_index=True)
print(df_all["label"].value_counts())
print(df_all.head())

y, labels = pd.factorize(df_all["label"])
X = df_all[FEATURES]

clf = DecisionTreeClassifier(max_depth=3, random_state=42)
clf.fit(X, y)

print(export_text(clf, feature_names=FEATURES))
print("Label mapping:", dict(enumerate(labels)))

y_pred = clf.predict(X)
print("Confusion matrix (train):\n", confusion_matrix(y, y_pred))
print(classification_report(y, y_pred, target_names=labels))
