# robot_hardware

물리 하드웨어 인터페이스 노드. `ament_python` 빌드.

## 노드 (계획)

| 노드 | 역할 | 토픽/파라미터 |
|---|---|---|
| `camera_csi_node` | CSI IMX219 GStreamer → `sensor_msgs/Image` | pub: `/camera_top/image_raw`, `/camera_body/image_raw` (각각 sensor-id 0/1) |
| `mcu_bridge_arm_node` | Arduino_arm (PCA9685 → MG996R×6) USB serial 브릿지 | sub: `/arm_command`, 포맷 `<θ1...θ6>` |
| `mcu_bridge_base_node` | Arduino_base (메카넘 4모터) USB serial 브릿지 | sub: `/base_command`, pub: `/wheel_odom` |

## 카메라 슬롯 매핑 (확정)
- **sensor-id=0** → 광각 (SMG IMX219 160° 8MP, 본체 상단 80cm)
- **sensor-id=1** → 본체 cam (Waveshare IMX219 B0183, 그리퍼 손목)

## 시리얼 포트
- `/dev/ttyACM0` 또는 `/dev/ttyUSB0` — udev rule로 고정 이름(`/dev/arm`, `/dev/base`) 부여 권장
