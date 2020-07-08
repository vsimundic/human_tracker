import pandas as pd
import numpy as np

dataset_data = pd.read_csv('//home/valentin/human_tracker_ws/FERIT_dataset/kinect_k07_1/results/k07_stamps_annotations.csv').to_numpy()
method_data = pd.read_csv('/home/valentin/human_tracker_ws/FERIT_dataset/kinect_k07_1/results/k07_full_method21_data.csv').to_numpy()

num_of_frames = dataset_data.shape[0]
print(num_of_frames)

TP = 0
TN = 0
FP = 0
FN = 0

if dataset_data.shape[0] == method_data.shape[0]:
    for i in range(dataset_data.shape[0]):
        ds_detections = dataset_data[i, 0]
        mtd_detections = method_data[i, 0]

        if ds_detections != 0 and ds_detections == mtd_detections:
            TP += ds_detections

        elif ds_detections == 0 and ds_detections == mtd_detections:
            TN += 1

        elif ds_detections < mtd_detections:
            TP += ds_detections
            FP += (mtd_detections - ds_detections)

        elif ds_detections > mtd_detections:
            TP += mtd_detections
            FN += (ds_detections - mtd_detections)

RECALL = TP/float(TP + FN)
PRECISION = TP/float(TP + FP)
ACCURACY = (TP + TN) / (TP + FP + TN + FN)
FRR = (100 * FN)/(TP + FN)
FPPF = FP / num_of_frames

print("recall: " + str(RECALL))
print("precision: " + str(PRECISION))
print("Accuracy: " + str(ACCURACY))
print("FRR = " + str(FRR))
print("FPPF = " + str(FPPF))
