# Robomaster Helios
位于master分支下的代码为所有兵种通用代码，通过运行程序时输入argv参数，控制不同相机驱动
## 1. 注意
- 运行：
	运行代码需制定内容包括，兵种（-hero -infantry -sentry -UAV -video），其中-video表示使用视频运行代码，目标颜色(-blue -red),调试选项(-origin -boxes -lamp 等等)，相机编号(-cameraIndex = ),视频路径(-path = )等，
	具体参数可在运行时添加 -help 参数查看。 

- 配置：
	本代码中所有所需链接库及头文件，除openCV相关文件外均置于项目文件夹内，但须注意，当使用DAHUA工业摄像头及Intel435D摄像头时，存在链接库平台问题，需及时替换链接库版本。
- 线程任务：
	![multi-threads](https://github.com/luojunhui1/BlogPicture/blob/master/Windows/threads%20(1).jpg)
    ![data flow](https://github.com/luojunhui1/BlogPicture/blob/master/Windows/GPUDATAMAP.jpg)