# HandGuidingExample
### 配置说明：
1. 在主目录下配置程序运行库的环境
   gedit ~/.bashrc
   在该文件最后加入：
   export LD_LIBRARY_PATH=/home/user/Desktop/HandGuidingExample/lib/lib32
   `注意：   home/user/Desktop是相对路径，应用时以实际的路径为准`。
   保存并并退出。
   
2. 重启一个新的terminal（终端）

3. 启动示教器软件，将IP地址配置成192.168.1.100

4. 在terminal中切换到**/HandGuidingExample/release目录

5. 运行 ./Handguidingexample


### 使用说明：
1. 本插件支持多种传感器类型，如果使用的是ATI的传感器，则应现在SensorType下拉框中选择该传感器，以此类推；
2. 支持三种拖动模式，分别是位姿、位置和姿态拖动；可以在Drag Mode中进行选择；
3. Calculate Method 和 Control Model里面的配置保持不变；
4. Tool Data里面的是传感器和工具（作为整体）相对于机械臂末端法兰的位置和姿态；
5. Sensor Data: 传感器的实时数据；
6. Force 和 Torque里面的数据是拖动时作用在工具端对的实时数据；
7. Control period和BuffSize Limit里面的数据使用默认值即可；
8. sensitivity即为传感器在6个方向的灵敏度，增大其值，拖动更轻松，但是可能引起不稳定，需要同时增加其阻尼参数；
9. damp即为6个方向的阻尼参数，拖动速度越大，阻尼效应越明显；
10. stiffnes即为拖动时的弹性系数，其值越大，弹簧的效果越明显，若想没有弹簧的效果，可将其值置为0；
11. threshold为拖动时的阈值，即实际值超过该数，传感器才会有响应；
12. limit：作用在传感器上的最大力和力矩，当超过其设定时，按设定的最大值计算。

### 操作说明：
1. 传感器在上电后，应该静置半小时以后再进行使用；
2. 使用前先对其进行标定，将传感器和工具安装好后，点击Pose1、pose2、pose3使其依次运动到相应的位置，然后对其进行标定，点击Calibrate按钮；
3. 标定后Sensor Data的值应该处于0附近，如若不是，需要重新标定；如若是，则可点击Start按钮，进行拖动；
4. 传感器如果静置一段时间，其输出数值一直漂移，则在使用一段时间后，对其进行重新校准（标定）；
5. 如果在运动过程中出现超速，则可点击Reset按钮，在点击Calibrate按钮后，对传感器进行恢复。

Maintainer status: **`under developing`**<br>
Maintainer: liug@aubo-robotics.cn | yuanziyi@aubo-robotics.cn<br>
The copyright of the source code is shared by above two developers.<br>

```diff
This software are licensed as LGPL software. In practice, this means that, when you distribute this software,you are
required to distribute any changes you make to this software as well under the same license. The derived work may be 
distributed under any license you see fit.
```
