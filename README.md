# spina_arm_controll
### Node and Topic
![](rosgraph.png)
## Setup
```bash
$ cd ~/ros2_ws/src  #Go to ros workspace
$ git clone https://github.com/iHaruruki/spina_arm_controll.git #clone this package
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source install/setup.bash
```
## Usage
### Simple command
```bash
sudo chmod 777 /dev/ttyUSB0
```
> [!NOTE]
> Please check USB port and change `launch/tatto_serial.launch.py` file.  
> USB portのを確認し，launchファイルを変更する
`launch/tatto_serial.launch.py`
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='spina_arm_controll',
            executable='serial_controller_node',
            name='serial_controller_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0', # Please check USB port
            }]
        ),
        
    ])
```
Build
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select apina_arm_controll
```
```bash
ros2 launch spina_arm_controll serial_controller.launc.py
```
**Sending angle command with command line interface tools**  
Set the overall angle to -90°
```bash
ros2 topic pub /angle_cmd std_msgs/msg/String "{ data: 'A0p-090' }" --once
```
If you want to control a single module, use `"{ data: 'C1p+015' }"`.
```bash
ros2 topic pub /angle_cmd std_msgs/msg/String "{ data: 'C1p-030' }" --once
```
**Details of sending angle command**    
Overall Control
* A: Overall Control / 全体制御
* 0: zero
* `p` or `r`: Attitude axis setting (p: Pitch, r: Yaw) / 姿勢の軸設定(p:Pitch, r:Yaw)
* -090: -90 degrees / -90度<br>

Module Individual Control
* C: Module Control / モジュール制御
* 1: Module number (1-6) / モジュール番号（1-6）
* `p` or `r`: Attitude axis setting (p: Pitch, r: Yaw) / 姿勢の軸設定(p:Pitch, r:Yaw)
* -015: -15 degrees / -15度<br>

### Sending angle command with Node
```bash
ros2 run spina_arm_controll angle_send_node --ros-args -p modules:="['r-15','p+10','r0','p5','r-20','p+30']"
```
- m1 → `r-015` （ヨー軸 –15°）
- m2 → `p+010` （ピッチ軸 ＋10°）
- m3 → `r+000` （ヨー軸 0°）
- m4 → `p+005` （ピッチ軸 ＋5°）
- m5 → `r-020` （ヨー軸 –20°）
- m6 → `p+030` （ピッチ軸 ＋30°）

### Sending angle with joy
```bash
# This command is not necessary if you are connecting via Bluetooth.
sudo chmod 777 /dev/ttyUSB0
```
```bash
ros2 launch spina_arm_controll joy_spina.launch.py
```
## License
## Authors
![Haruki Isono](https://github.com/iHaruruki)
## References
