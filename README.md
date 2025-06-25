# LIDSOR: A Filter for Removing Rain and Snow Noise Points from LIDAR Point Clouds in Rainy and Snowy Weather

### [Paper](https://isprs-archives.copernicus.org/articles/XLVIII-1-W2-2023/733/2023/)

> LIDSOR: A Filter for Removing Rain and Snow Noise Points from LIDAR Point Clouds in Rainy and Snowy Weather

> He Huang, [Xinyuan Yan](https://naclzno.github.io/Xinyuan-Yan/), Junxing Yang $\ddagger$, Yuming Cao $\dagger$, and Xin Zhang

$\dagger$ Project leader $\ddagger$ Corresponding author

## Highlights

- **We performed a comprehensive statistical analysis on the intensity and spatial distribution characteristics of noise points generated in rain and snow, subsequently employing gamma curves for a more precise fitting of these noise point distribution characteristics.**
- **We propose a LIDSOR filter. This filter is built on intensity and distance thresholds.**



## Data Acquisition and Label

### Platform

<p align="center">
    <img src="./figures/platform.png" alt="overview" width="70%">
</p>

### Camera (Snow)

![camera](./figures/camera.jpg)

### Rosbag Package (Snow)

<p align="center">
    <img src="./figures/points.gif" alt="overview" width="50%">
</p>

### Point-wise Label (Snow)

<p align="center">
    <img src="./figures/label.png" alt="overview" width="50%">
</p>

## Abstract

As autonomous driving technology advances, ensuring the system's safety in rain and snow has emerged as a pivotal research topic. In rainy and snowy weather, rain and snow can generate noise points within the point cloud captured by the Light Detection and Ranging (LiDAR), significantly impeding the LiDAR's sensing capability. To address this problem, we first manually label the point cloud data gathered in rain and snow, categorizing all points into noise points and non-noise points. Subsequently, we analyze the intensity and spatial distribution characteristics of the rain and snow noise points and employ the gamma distribution curve to illustrate the spatial distribution characteristics of these noise points. Finally, we propose a Low-Intensity Dynamic Statistical Outlier Removal (LIDSOR) filter, an enhancement of the existing Dynamic Statistical Outlier Removal (DSOR) filter. Experimental results suggest that the LIDSOR filter can effectively eliminate rain and snow noise points while preserving more environmental feature points. Additionally, it consumes fewer computational resources. The filter we propose in this paper significantly contributes to the safe operation of the autonomous driving system in diverse complex environments.

![network](./figures/network.jpg)

## Quantitative Results

![results_light](./figures/results.png)


## Directory Structure and Data Format
### Directory Structure:
- `og`: Original point cloud data
- `sn`: Ground truth non-noise points
- `sp`: Ground truth noise points
- `rt`: Classification results
    - `en`: Predicted non-noise points
    - `ep`: Predicted noise points
```
LIDSOR
├── data/snow
│   ├── og/
│   │   ├── H1.txt/
│   ├── sn/
│   │   ├── H1.txt/
│   ├── sp/
│   │   ├── H1.txt/
│   ├── rt/
│   │   ├── LIDSOR/
│   │   │   ├── en/
│   │   │   ├── H1.txt/
│   │   │   ├── ep/
│   │   │   ├── H1.txt/
```
### Data Format:
```
H1.txt
X, Y, Z, Intensity, Label, Time
92.724 106.471 -3.54917 3 0 3
49.2686 66.7579 -2.88287 5 0 3
48.9762 66.8489 -2.8794 2 0 3
48.7731 67.0615 -2.88121 2 0 3
48.5621 67.2391 -2.8819 2 0 3
```

## How to build & Run

------

   ```
   $ cd lidsor && mkdir build && cd build
   $ cmake ..
   $ make -j 16
   ```

<!-- ## When will we publish the source code?

The source code for the Low-Intensity Dynamic Statistical Outlier Removal (LIDSOR) filter is scheduled to be published by 2025. We kindly ask for your patience. -->

## Contact
If you have any questions, please contact yan1075783878@gmail.com

## Citation

If you find this project helpful, please consider citing the following paper:
```
@article{huang2023lidsor,
  title={LIDSOR: A filter for removing rain and snow noise points from LiDAR point clouds in rainy and snowy weather},
  author={Huang, He and Yan, Xinyuan and Yang, Junxing and Cao, Yuming and Zhang, Xin},
  journal={The International Archives of the Photogrammetry, Remote Sensing and Spatial Information Sciences},
  volume={48},
  pages={733--740},
  year={2023},
  publisher={Copernicus GmbH}
}
```

