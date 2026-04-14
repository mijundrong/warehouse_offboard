# warehouse_offboard

ROS 2 Humble, Gazebo, PX4 SITL 환경에서 창고 월드 기반 드론 오프보드 미션을 실행하기 위한 패키지입니다.

---

## External Dependencies

이 저장소에는 아래 항목들이 포함되어 있지 않습니다. 별도로 설치해야 합니다.

- PX4-Autopilot
- px4_msgs
- QGroundControl
- MicroXRCEAgent

---

## Build

코드를 수정한 경우에는 다시 build 해야 합니다.

~~~bash
cd ~/ros2_ws
colcon build --packages-select warehouse_offboard
source install/setup.bash
~~~

처음 새로운 PC에서 workspace를 세팅할 때는 ROS 2 underlay를 먼저 source 하는 것이 좋습니다.

~~~bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select warehouse_offboard
source install/setup.bash
~~~

---

## Before Running

ROS 2 패키지를 실행하는 터미널에서는 항상 아래 명령을 먼저 실행합니다.

~~~bash
cd ~/ros2_ws
source install/setup.bash
~~~

---

## Run Sequence

아래 순서대로 총 6개의 터미널을 사용합니다.

### Terminal 1: QGroundControl

~~~bash
cd ~/Downloads
./QGroundControl-x86_64.AppImage
~~~

### Terminal 2: Gazebo

~~~bash
cd ~/ros2_ws/src/warehouse_offboard
source ~/.bashrc
gz sim -r ~/ros2_ws/src/warehouse_offboard/worlds/warehouse.sdf
~~~

### Terminal 3: PX4

~~~bash
cd ~/PX4-Autopilot
source ~/.bashrc
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="-10,0,0.3,0,0,0" ./build/px4_sitl_default/bin/px4
~~~

### Terminal 4: MicroXRCEAgent (ROS ↔ PX4 bridge)

~~~bash
MicroXRCEAgent udp4 -p 8888
~~~

### Terminal 5: goto_point.py 실행 (드론 미션 백엔드)

~~~bash
cd ~/ros2_ws
source install/setup.bash
ros2 run warehouse_offboard goto_point --ros-args --params-file ~/ros2_ws/src/warehouse_offboard/params/goto_point.yaml
~~~

### Terminal 6: chat_mission_ui 실행

~~~bash
cd ~/ros2_ws
source install/setup.bash
ros2 run warehouse_offboard chat_mission_ui
~~~

---

## Notes

- 코드를 수정한 뒤에는 반드시 다시 build 해야 합니다.
- `goto_point.yaml` 파일에서 waypoint 관련 파라미터를 수정할 수 있습니다.
- QGroundControl AppImage 위치가 다르면 경로를 맞게 수정해야 합니다.
- PX4 경로가 `~/PX4-Autopilot`이 아닐 경우 해당 경로로 수정해야 합니다.
- `llm_selector.py`는 `OPENAI_API_KEY` 환경변수를 사용하는 방식입니다. 필요 시 환경변수를 먼저 설정해야 합니다.

예시:

~~~bash
export OPENAI_API_KEY="your_api_key"
~~~
