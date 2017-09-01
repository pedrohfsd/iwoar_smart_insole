# iWOAR Smart Insole
Repository for the iWOAR 2017 research paper: 
> "Bottom-up Investigation: Human Activity Recognition Based on Feet Movement and Posture Information"

This repository holds the dataset generated from each of the 11 subjects used in the study with roughly 800,000 samples. Each dataset contains some of the features originally generated from the sensor's raw signals begore applying [HALL's Correlation-based feature selection](https://www.lri.fr/~pierres/donn%E9es/save/these/articles/lpr-queue/hall99correlationbased.pdf).

## Samples
Each line/sample is generated from a tumbling window of 300 milliseconds. Each signal contains the values collected from all sensors at that given time. Samples are already sorted in ascending order by the time they were collected.

## Features
* d_mean - mean distance from the ground in the current window
* b_cum_diff - cumulative difference from previous windows in that activity group
* b_var - barometer variance in the current window
* b_mean - barometer mean in the current window
* a_x_mean - accelerometer x axis mean in the current window
* a_y_mean - accelerometer y axis mean in the current window
* a_z_mean - accelerometer z axis mean in the current window
* g_x_mean - gyroscope x axis mean in the current window
* g_y_mean - gyroscope y axis mean in the current window
* g_z_mean - gyroscope z axis mean in the current window
* m_x_mean - magnetometer x axis mean in the current window
* m_y_mean - magnetometer y axis mean in the current window
* m_z_mean - magnetometer z axis mean in the current window
* fsr_fsr0_mean - mean value for the front-left pressure sensor in the current window
* fsr_fsr1_mean - mean value for the front-center pressure sensor in the current window
* fsr_fsr2_mean - mean value for the front-right pressure sensor in the current window
* fsr_fsr3_mean - mean value for the center pressure sensor in the current window
* fsr_fsr4_mean - mean value for the rear-left pressure sensor in the current window
* fsr_fsr5_mean - mean value for the read-right pressure sensor in the current window
* af_r_mean - mean Euler angle of roll in the current window
* af_p_mean - mean Euler angle of pitch in the current window
* activity - label specifying the current activity
