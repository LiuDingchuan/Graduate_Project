<!--
 * @Description: 
 * @Version: 2.0
 * @Author: Dandelion
 * @Date: 2023-05-11 12:31:34
<<<<<<< HEAD
 * @LastEditTime: 2024-07-24 19:04:58
=======
 * @LastEditTime: 2023-07-15 18:27:45
>>>>>>> 0792d8e368bad9424ecb953d3755c115bcde5f80
 * @FilePath: \Graduate_Project\README.md
-->
# 双轮腿式机器人控制系统设计

本开源项目idea来自于哈尔滨工程大学创梦之翼战队[RoboMaster平衡步兵机器人控制系统设计](https://zhuanlan.zhihu.com/p/563048952)。本人所做的主要工作是将其理论用代码复现，并用webots仿真软件成功验证其理论可行性。现将工程全部开源，方便大家交流。

## 使用软件

|软件|版本|用途|备注|
|---|---|---|---|
|SolidWorks|2022|简易轮腿机器人建模、地形建模|不要使用低于2022版本|
|Webots|2020|仿真软件|不要使用其他版本|
|VSCode||代码编辑器|需要配置.json文件|
|Visual Studio|2022|debug调试、代码编辑|需要配置编译环境|
|matlab|2022a|离线lqr参数计算与拟合|需要根据报错提示配置一些工具箱|
|eigen库|3.4.0|矩阵计算|C++附加库，官网下载即可|

## 环境配置

### eigen库

由于lqr计算中涉及高维矩阵，需要用到C++的eigen库，其下载和后续的环境配置详见[Webots：VSCode作为控制器IDE并调用Eigen库](https://blog.csdn.net/qq413886183/article/details/124692107?spm=1001.2014.3001.5501)
由于这个文章需要CSDN会员，所以这里简单介绍一下：

1. 下载eigen库最新的stable版
2. 打开webots/resources/Makefile.include，搜索“Linked libraries”，加入
   ```
   INCLUDE = -I"(你的Eigen安装目录)/Eigen"
   ```
3. webots中编译，不会报错即为成功

### VSCode

为了让VSCode中对代码进行修改时具备自动补全功能，需要在c_cpp_properties.json文件中配置Eigen库路径和Webots库路径。
    ```
    "includePath": [
        "${workspaceFolder}",
        "E:\\Webots2020\\include\\controller\\cpp",
        "E:\\Webots2020\\include\\controller\\c",
        "E:\\eigen-3.4.0\\Eigen"
        ]
    ```

### Visual Studio

请参考[Webots仿真: 使用Visual Studio作为IDE并Debug调试](https://zhuanlan.zhihu.com/p/621739488)

## 仿真使用介绍

将main分支下载到本地后，解压，进入worlds文件夹。
test_world.wbt——平地试验功能
起伏路段——带有斜坡和弧度坡的环境
起伏路段_双腿——带有大宽度的斜坡，可以供双腿通过

进入仿真世界后，如果是想使用webots自身的IDE进行调试，将controller项选择为"dynamic_lqr"即可；
如果是想使用Visual Studio作为IDE进行调试并debug，需要将controller项选择为"extern",并打开...\controllers\Dynamics_LQR_onVS中的.sln文件，先运行程序，再运行仿真，即可开始debug。

仿真运行后的功能按键映射如下：
|按键|功能|
|---|---|
|↑|前进|
|↓|后退|
|←|逆时针旋转|
|→|顺时针旋转|
|W|高度升高|
|S|高度降低|
|A|左倾|
|D|右倾|

## 调参说明

进入”lqr_matlab离线调参"文件夹，打开"model_LQR.m"文件和"count_coeff.m”文件，在model_LQR.m的最后，可以对Q矩阵和R矩阵的参数进行调整。再在count_coeff.m文件中运行程序，运行完毕后将工作区中的参数K_lib打开，将12*20的表格中的数据复制到MyRobot.cpp的K_coeff中，即可实现离线lqr计算调参。

