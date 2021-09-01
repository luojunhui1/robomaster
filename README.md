# Robomaster Helios
本仓库为西南交通大学Helios战队2020视觉组所有成员所建，用于管理用于Robomaster赛场上哨兵、英雄、步兵的视觉代码。
***
## 硬件列表及文件树
- 硬件列表

|  硬件   | 型号  |
|  ----  | ----  |
| 视觉开发板  | NVIDIA XAVIER NX |
| 视觉开发板  | NVIDIA JETSON NANO |
| 视觉开发板  | DJI MANIFOLD 1 |
| 摄像头  | DAHUA A5121CU210 |
| 摄像头  | INTEL D435 |
| 摄像头  | KS2A543 |
| 摄像头  | KS1A522 |
- 二级文件树列表

<pre><font color="#3465A4"><b>.</b></font>
├── <span style="background-color:#4E9A06"><font color="#3465A4">Armor</font></span>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">include</font></span>
│   ├── <font color="#3465A4"><b>resource</b></font>
│   └── <span style="background-color:#4E9A06"><font color="#3465A4">src</font></span>
├── <font color="#4E9A06"><b>CMakeLists.txt</b></font>
├── <span style="background-color:#4E9A06"><font color="#3465A4">Drivers</font></span>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">DAHUA</font></span>
│   ├── <font color="#4E9A06"><b>Driver.h</b></font>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">RealSense</font></span>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">V4L2KAS</font></span>
│   └── <span style="background-color:#4E9A06"><font color="#3465A4">VideoDriver</font></span>
├── <span style="background-color:#4E9A06"><font color="#3465A4">Filter</font></span>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">include</font></span>
│   └── <span style="background-color:#4E9A06"><font color="#3465A4">src</font></span>
├── <span style="background-color:#4E9A06"><font color="#3465A4">Log</font></span>
│   ├── <font color="#4E9A06"><b>log.txt</b></font>
│   └── <font color="#4E9A06"><b>video_count_file.txt</b></font>
├── <font color="#4E9A06"><b>main.cpp</b></font>
├── <span style="background-color:#4E9A06"><font color="#3465A4">Other</font></span>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">include</font></span>
│   └── <span style="background-color:#4E9A06"><font color="#3465A4">src</font></span>
├── <span style="background-color:#4E9A06"><font color="#3465A4">Pose</font></span>
│   ├── <font color="#4E9A06"><b>camera.xml</b></font>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">include</font></span>
│   └── <span style="background-color:#4E9A06"><font color="#3465A4">src</font></span>
├── <font color="#4E9A06"><b>README.md</b></font>
├── RMSelfStart.sh
├── <span style="background-color:#4E9A06"><font color="#3465A4">Serials</font></span>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">include</font></span>
│   └── <span style="background-color:#4E9A06"><font color="#3465A4">src</font></span>
└── <span style="background-color:#4E9A06"><font color="#3465A4">Thread</font></span>
    ├── <span style="background-color:#4E9A06"><font color="#3465A4">include</font></span>
    └── <span style="background-color:#4E9A06"><font color="#3465A4">src</font></span>
</pre>

## 代码效果展示

## 特点
- 运行：参考上交2018代码写法，通过运行可执行程序时输入参数，控制代码的执行，相比较于预编译的写法而言，在使用上更加方便，也更易于调试。可输入的参数包括：
	- 兵种（-hero -melee -track -sentry -UAV -video），其中-video表示使用视频运行代码
	- 目标颜色(-blue -red)
	- 调试选项(-origin -box -l 等等)
	- 相机编号(-cameraIndex = )
	- 打开视频路径(-path= )，
	具体参数可在运行时添加 -help 参数查看。 

- 统一的相机驱动接口：我们使用了包括支持UVC协议的USB摄像头，Intel D435双目摄像头，大华A5131CU210工业摄像头在内的多种摄像头，这在一定程度上造成了我们在不同机器人上往往需要采取不同的摄像头驱动，为代码的移植更新带来了不便，为此我们提供统一的接口driver(虚函数)来根据机器人的种类选择不同的驱动，一定程度上减小了多种摄像头带来的困扰。

- 更新的预处理方案: 采取先按灰度值二值化的方案，再根据红蓝通道差值进行灯条颜色判断的方案，在红蓝白三种颜色高灰度物体间能稳定地区分，同时能保证灯条的形态真实。

- 更新的装甲板匹配策略：在装甲板匹配中采取优先级策略和二次筛选策略。

- 运动控制逻辑：运动控制逻辑除了kalman预测外还加入了消抖自稳和pitch轴补偿。
 
- 多线程：线程分为图像产生线程，神经网络线程，能量机关线程，装甲板识别线程和反馈接收线程。
 
- GPU加速： 由于代码的并行度并不高，GPU加速效果不明显，所以GPU加速方案主要用于神经网络。

## 2.功能实现说明
### 2.1 传统检测算法
#### 2.1.1 灯条识别
由于相机的问题，我们始终未能将相机的成像调制一个较为完美的地步，使得我们能在图像预处理阶段（使用颜色空间包括BGR,HSV,HSI等）就能将红蓝灯条分别以较好的形态完整从图像中提取出来，实际上当我们这样尝试时，较为普遍的情况是二值化后的灯条中心呈现黑色缺口，这是由于灯条中心在图像成像中显现出白色特征的原因，在各个颜色空间上的值都比较高；并且这种方法提取出的灯条在形态上与图像上灯条的形态相比更为“臃肿”，这会对之后需要使用到灯条形态的功能（灯条筛选，灯条匹配，大小装甲板判别，Pnp测距等）造成一定影响。为此，我们使用“先提取灯条，再区分颜色”的方法来改善传统方法，并且由于ROI的应用和GPU的使用，这种方法即使在图像分辨率较高时，对任务执行速度的影响也微乎其微。

它的基本思想为：**通过灯条目标区域红色通道减去蓝色通道的值的平均值，来判断灯条的颜色**，仅仅只将通道相减的步骤用在了之后的数值计算中，就带来了如下的改观：
- 使用灰度图二值化，得到的灯条形态更真实
- 红、蓝、白色的R通道与B通道的平均差值有着明显的区别，在实际测试中，红蓝灯条中的平均差值在±60左右，而白色物体明显在0左右
- 尺度不变性，由于使用的是平均值进行判断，当装甲板距离变化，灯条大小变化时，该方法仍然有效

#### 2.1.2 灯条匹配
在灯条匹配的过程中，最重要的有如下几点：
1. 灯条匹配参数的选择
2. 灯条匹配参数的调整
3. 匹配灯条对的选择
对于第一点，我们最终选择参数如下：
```c++
	float maxArmorAngle = 45;//装甲板最大倾斜角度
        float maxAngleError = 6;//装甲板两灯条最大角度差
        float maxLengthError = 0.70;//装甲板灯条最大允许高度差
        float maxDeviationAngle = 50;//装甲板两灯条最大偏差角
        float maxYDiff = 2;//装甲板灯条y周最大偏差率
        float maxRatio = 20;//装甲板最大允许长宽比
        float minRatio = 0.6;//装甲板最小允许长宽比

        float minLightArea = 100;//灯条最小允许像素面积
        float maxLightArea = 2500;//灯条最大允许像素面积
        float maxLightAngle = 60;//灯条最大允许倾斜角
        float minLightW2H = 0.5;//灯条最小允许长宽比
	float maxLightW2H = 40;//灯条最大允许长宽比
```
而对于第二点，有一些参数可以在程序运行过程中动态更新，而大多数参数的数值选择，仅使用通过在测试中不断调整得到的数值即可。
对于匹配灯条对的选择，大多数队伍都采用了优先级的方案，我们通过将灯条匹配对的各项匹配参数输入到优先级计算公式中，得到一个初级的装甲板优先级向量；在实际测试过程中，我们也发现，排除未识别到装甲板的情况，在大多数误识别装甲板的情况中，有一个灯条往往是真实装甲板的两个灯条之一，故在原有装甲板优先级向量中再根据首个最可能装甲板的灯条再调整一次位置，将其灯条组成的“疑似装甲板”在装甲板优先向量中前移，得到一个新的装甲板优先向量，在其中选择前K个“疑似装甲板”，通过数字识别或者神经网络，选择最后可能的那个装甲板。

#### 2.1.3 ROI更新
对于未使用GPU加速或者图像分辨率过大（如1920*1240）的传统算法来说，在预处理阶段不使用ROI将会对代码性能带来极大的降低。在选择ROI时，由于识别部分效果较好，我们并不需要过于复杂的ROI区域控制逻辑，而只是每次根据识别目标装甲板的位置和大小对ROI区域进行更新即可，当未找到目标装甲板时，将ROI区域迅速（经过2-3次识别过程）恢复到全图区域。

### 2.2 姿态解算
姿态计算使用使用Pnp算法，在工程的的Pose/SolveAngle模块中，OpenCV提供了solvePnP函数借口，但不同的Pnp算法的适用条件和效果都不太相同，具体可以参见：https://docs.opencv.org/4.4.0/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d,
Pnp的原理部分可以参见
https://blog.csdn.net/KYJL888/article/details/81319422?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-3.channel_param&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-3.channel_param

### 2.3 能量机关
能量机关Energy类通过接口函数 ```EnergyTask(Mat& src)``` 调用能量机关识别。该接口函数由以下功能函数顺序执行，最终计算出预测打击坐标```predict_point```。
#### 2.3.1  **detectArmor**
该函数用以检测所有装甲。调用```isValidArmorContour```来判断找到的装甲尺寸是否合格。将所有合理的装甲板传入Energy类成员```armors```中。
#### 2.3.2  detectFlowStripFan
该函数用以检测所有可能的流动条所在的扇叶。调用```isValidFlowStripFanContour```来判断找到的扇叶尺寸是否合格。将所有合理的装甲板传入Energy类成员```flow_strip_fans```中。
#### 2.3.3  detectR
该函数用以确定唯一中心点```centerR```。 首先检测所有可能的中心R，然后调用```isValidCenterRContour```进行初筛，再通过判断是否与```flow_strip_fans```有重合部分作二次筛选。若有多个候选R，通过比较与上一帧R距离大小打分，筛选出唯一中心点```centerR```，确定R中心坐标（作为风车旋转中心点）```circle_center_point```。
#### 2.3.4 getTargetPoint
该函数用以确定目标装甲板。在本算法中主要以两大因素评判装甲板：
- 装甲板与流动条所在扇叶交集面积    

- 装甲板与上一次目标装甲板距离

选取得分最高的装甲板传入```target_armor```，将其中心点传入```target_point```
#### 2.3.5  getPredictPoint
1. 转为极坐标系。先将目标打击点转为极坐标```target_polar_angle```便于后续角度计算。
2. 执行大小符提前角度计算。

- 大符模式
创建Energy类成员```target_angle_1second```记录一秒内目标极坐标。
通过```target_angle_1second```里的部分值计算角速度```v0_rate```。再判断该角速度是否合理。若不合理，用预测速度替代测量速度，使用预测速度算出提前角：
```c++
    if ((v0_rate < 0.52) || (v0_rate > 2.09)) {
        t1_real = getTickCount();
        t1 = (t1_real - t0_real)*0.000000001 + t0;     //ms->s
        predict_rad_norm = calPreAngle(t1, t1 + shoot_Debug);    //用预测速度替代测量速度
    }
```
若合理，使用当前速度算出提前角
```c++
    else{
            float wt = asin(((v0_rate) - 1.305) / 0.785);       //wt--rad/s ,[-1,1]
            if (wt > 0) {
                if (v0_rate_1 > v0_rate_2)
                    wt += 90 * (PI / 180);      //90~180
            } else {
                if (v0_rate_1 > v0_rate_2)
                    wt -= 90 * (PI / 180);      //-90~-180
            }
            t0_real = getTickCount();       //获得开始点时间
            t0 = wt / OMEGA;      //映射到速度三角函数里
            predict_rad_norm = calPreAngle(t0, t0 + shoot_Debug);
        }
```
在大符模式中，通过调用```calPreAngle(float start_time, float end_time) ```计算提前角。该函数根据官方给出的正弦速度曲线以及总射击时间计算出射击提前角。所以只需在找到第一个目标点时确定```t0_real```以及其角度映射在正弦速度曲线上的```t0```，后续任务中获取```t1_real```即可确定该时刻在速度曲线上的```t1```，调用```calPreAngle```即可获得此时刻的提前角```predict_rad_norm```。
- 小符模式
由于小符模式速度恒定，即每秒角速度一定，预测提前角度即为射击时间与小符转速的乘积。

- 解算回直角坐标系

确定提前角后，结合旋转方向将其与目标极坐标做加减运算。再将其转回直角坐标系，解算回```predict_point```。



### 2.4 神经网络
神经网络的应用上，使用YOLOv4和YOLOv4-Tiny作为主要的网络作为尝试，相比与传统的视觉算法来说，神经网络的优点在于它的高效和能够更深入地调用我们所选
用的硬件平台的计算资源。我们曾尝试过对传统视觉方法进行一定的CUDA加速，但是由于任务中需要并行的部分并不多，导致加速效果其实微乎其微。

但如果使用神经网络进行装甲板或者车体的识别，可以将整个模型推理的过程结合NVIDIA生态的DEEPSTREAM平台进行加速，可能效果会好很多，可能未来比赛场会上以
神经网络为主，传统方法为辅的视觉处理结构为主流。

鉴于我们对神经网络的使用很粗糙，且没有训练模型的一些合适的数据，导致实际使用效果并不理想，因此这部分并不展开说明，具体使用参见Detector类中的`ModelDetectTask`。

### 2.6 多线程及自启动
多线程上使用C++11开始提供的thread头文件，通过创建多个线程来同时执行多个耦合程度较小的任务，通过锁机制和通信量管理任务流程的进行，合理分配计算和数据资源，避免线程间的混乱执行。项目的线程包括：
- 图像采集线程
- 装甲板检测线程
- 能量机关线程
- 神经网络线程
- 反馈接收线程
其中装甲板检测、能量机关和神经网络线程可以并行执行，这也是多线程起作用的主要原因之一。

![multi-threads](https://github.com/luojunhui1/BlogPicture/blob/master/Windows/threads%20(1).jpg)
(图例仅为设计图，实际实现并不与图中说明完全一致)
## 注意
本代码中所有所需链接库及头文件，除openCV相关文件外均置于项目文件夹内，但须注意，当使用DAHUA工业摄像头及Intel435D摄像头时，存在链接库平台问题，需及时替换链接库版本。
