<!--
 * @Description: 
 * @Version: 2.0
 * @Author: Dandelion
 * @Date: 2023-05-11 12:31:34
 * @LastEditTime: 2023-07-15 16:25:15
 * @FilePath: \webots_sim\README.md
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

由于lqr计算中设计高维矩阵，需要用到C++的eigen库，其下载和后续的环境配置详见[Webots：VSCode作为控制器IDE并调用Eigen库](https://blog.csdn.net/qq413886183/article/details/124692107?spm=1001.2014.3001.5501)

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

