import pandas as pd
import numpy as np

dataset_data = pd.read_csv('/home/valentin/human_tracker_ws/FERIT_dataset/kinect_k07_1/results/k07_stamps_annotations.csv').to_numpy()
method_data = pd.read_csv('/home/valentin/human_tracker_ws/FERIT_dataset/kinect_k07_1/results/k07_method21.csv').to_numpy()

# print(method_data)

method_full_data = dataset_data.copy()
counter = 0

for i in range(method_full_data.shape[0]):
    row_dataset = method_full_data[i]
    for row_method in method_data:
        if abs(row_dataset[1] - row_method[1]) < 0.00001:
            counter += 1
            method_full_data[i] = row_method[0:2]
            break


print(counter)
pd.DataFrame(method_full_data, columns=["PersoneNumber", "TIME"]).to_csv("/home/valentin/human_tracker_ws/FERIT_dataset/kinect_k07_1/results/k07_full_method21_data.csv", index=None)
