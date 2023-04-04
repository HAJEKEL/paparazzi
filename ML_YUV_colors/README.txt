This folder contains 2 files in .ipynb format.
- ML_labeling_and_results which contains functions to create labels for the test footage to train the Machine Learning model with, to find the YUV color thresholds for different obstacles.
- yuv_filter_test which contains the “extract_horizon” in Python, which extracts the horizon of the image using roll and pitch values. This file also helps perform tests to determine how effective different color filters are. By applying them all at the same time to an image and calculating whether the drone would evade them or not using our logic. 
