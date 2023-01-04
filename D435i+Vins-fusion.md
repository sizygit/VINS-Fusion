# D435i+Vins-fusion

## realsense SDK + VINS-Fusion

可以运行一下代码来实现在D435i上跑vins-fusion

```ros
roslaunch vins vins_rviz.launch
rosrun vins vins_node ~/vinsfusion_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
roslaunch realsense2_camera rs_camera_mine.launch
rosrun loop_fusion loop_fusion_node ~/vinsfusion_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

运行数据集

```ros
roslaunch vins vins_rviz.launch
rosrun vins vins_node ~/vinsfusion_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml 
(optional) rosrun loop_fusion loop_fusion_node ~/vinsfusion_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml 
rosbag play /home/szy/Documents/DataSet/MH_02_easy.bag
```

打印相关话题与节点图

```
rostopic echo /vins_estimator/path
rosrun rqt_graph rqt_graph
```

EVO评估

```
evo_traj euroc data.csv --save_as_tum
evo_rpe tum vio_loop.csv vio.csv -r full -va --plot --plot_mode xyz
evo_traj tum vio_loop.csv vio.csv --ref=vio.csv -p --plot_mode=xyz --align --correct_scale
evo_traj tum vio_loop.csv vio.csv --ref=data.tum-p --plot_mode=xyz --align --correct_scale
evo_traj bag /home/szy/Documents/DataSet/MH_02_easy.bag /imu --ref=/leica/position -va -p --plot_mode=xyz
```



## Vins-fusion原理

主程序为`VINS-Fusion/vins_estimator/src/estimator/rosNodeTest.cpp`

---

`cv::calcOpticalFlowPyrLK()`金字塔 [Lucas-Kanade光流法](https://blog.csdn.net/codedoctor/article/details/79175683) 进行角点跟踪-前端光流跟踪

`cv::goodFeaturesToTrack()`[Harris角点](https://lsxiang.github.io/Journey2SLAM/computer_vision/Harris/)与Shi Tomasi角点检测检测

`FeatureTracker::undistortedPts()`[针孔相机模型](https://www.cnblogs.com/wangguchangqing/p/8151128.html)下,像素平面与相机成像平面的转换

---

processMeasurements()中进行IMU预积分与追踪特征点之间的处理

`Estimator::initFirstIMUPose()`利用Eigen::Quaterniond相关函数将初始加速度对其重力场同时使相机Z轴对准重力加速度方向

`Estimator::processIMU()`对IMU的数据进行预积分处理,其中预积分class的push_back()的propagate()进行主要操作,涉及到了IMU的传播方针和协方差矩阵.雅克比矩阵等等.哪里不懂可以VIO的理论知识。

`Estimator::processImage()`寻找关键帧,进行初始化;[对极几何](https://blog.csdn.net/cindy9608/article/details/113765695?spm=1001.2101.3001.6650.2&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-2-113765695-blog-126859710.pc_relevant_3mothn_strategy_and_data_recovery&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-2-113765695-blog-126859710.pc_relevant_3mothn_strategy_and_data_recovery&utm_relevant_index=3)、三角化SVD、PNP、求解陀螺仪偏置、[视觉惯性对齐](https://blog.csdn.net/iwanderu/article/details/104672579)、

---



```cpp
vins_estimator/src/featureTracker/feature_tracker.cpp
/**
     * 跟踪一帧图像，提取当前帧特征点
     * 1、用前一帧运动估计特征点在当前帧中的位置，如果特征点没有速度，就直接用前一帧该点位置
     * 2、LK光流跟踪前一帧的特征点，正反向，删除跟丢的点；如果是双目，进行左右匹配，只删右目跟丢的特征点
     * 3、对于前后帧用LK光流跟踪到的匹配特征点，计算基础矩阵，用极线约束进一步剔除outlier点（代码注释掉了）
     * 4、如果特征点不够，剩余的用角点来凑；更新特征点跟踪次数
     * 5、计算特征点归一化相机平面坐标，并计算相对与前一帧移动速度
     * 6、保存当前帧特征点数据（归一化相机平面坐标，像素坐标，归一化相机平面移动速度）
     * 7、展示，左图特征点用颜色区分跟踪次数（红色少，蓝色多），画个箭头指向前一帧特征点位置，如果是双目，右图画个绿色点
    */
```

$SO(3)与so(3)$有
$\dot{R(t)}=\phi^\wedge(t)R(T)$

$R=exp(\phi^\wedge)=cos\theta\mathbf{I}+(1-cos\theta)\mathbf{\alpha\alpha^T}+sin\theta\mathbf\alpha^\wedge$

$\phi=ln(R)^{V}$

$ SE(3)与se(3)$有

hession矩阵:一个多元函数的二阶偏导数构成的方阵

因子图:

马氏距离:

IMU预积分:积分下一个时刻的PVQ作为视觉初始值;相邻帧PVQ变化量作为IMU约束;计算IMU误IMU误差协方差.


### 点云(PCL库)

单点特征:三维坐标（X, Y, Z）, 回波强度 Intensity, 法线 （Nx，Ny，Nz），主曲率（PCx, PCy, PCz, 及两个特征值 PC1, PC2）

局部特征:

PFH(Point Feature Histograms)

| Feature Name | Supports Texture / Color | Local / Global / Regional | Best Use Case                                                |
| ------------ | ------------------------ | ------------------------- | ------------------------------------------------------------ |
| PFH          | No                       | L                         |                                                              |
| FPFH         | No                       | L                         | 2.5D Scans (Pseudo single position range images)             |
| VFH          | No                       | G                         | Object detection with basic pose estimation                  |
| CVFH         | No                       | R                         | Object detection with basic pose estimation, detection of partial objects |
| RIFT         | Yes                      | L                         | Real world 3D-Scans with no mirror effects. RIFT is vulnerable against flipping. |
| RSD          | No                       | L                         |                                                              |
| NARF         | No                       | L                         | 2.5D (Range Images)                                          |
| ESF          | No                       | G                         |                                                              |



## C++

map、	

std::thread

## EVO

```
evo_ape - 用于评估绝对位姿误差；
evo_rpe- 用于评估相对位姿误差；
evo_traj - 这个主要是用来画轨迹、输出轨迹文件、转换数据格式等功能；
evo_res- 比较来自evo_ape或evo_rpe生成的一个或多个结果文件的工具；
evo_fig - （实验）工具，用于重新打开序列化图（使用–serialize_plot保存）；
evo_config - 这个主要用于evo工具全局设置和配置文件操作

```

