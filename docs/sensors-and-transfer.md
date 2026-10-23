# 센서 구성과 실물 전이 데이터 계약

실물 RB-Y1을 실행하지 않고 실제로 관측 가능한 입력으로 시뮬레이션 태스크를 수행한다. 장비 드라이버와 실측 보정이 필요한 지점을 데이터로 명시하며, 장비를 연결하기만 하면 동작한다는 주장을 하지 않는다.

## 공식 확인과 프로젝트 선택

| 항목 | 공식 근거 | 이 프로젝트의 선택 |
|---|---|---|
| 로봇 | 비전 카메라는 기본 제공되지 않는다. V1.2는 양팔 FT 구성, 그리퍼·목은 옵션이다. | Menagerie M형 V1.2·기본 그리퍼·목 모델 |
| 머리 | ZED X Mini는 머리·손목 장착 가능한 공식 비전 옵션이다. | 머리 ZED X Mini wide 대응 RGB+깊이 |
| 손목 | D405는 공식 비전 옵션이며 RGB와 stereo depth를 제공한다. | 왼쪽·오른쪽 D405 대응 RGB+깊이 |
| D405 | 0.07–0.50 m, depth H87°/V58°, 최대 720p 30 FPS, USB, 42×42×23 mm | 범위 밖 깊이는 NaN. 640×360 pinhole·수직 FOV 58° |
| 머리 깊이 | ZED X Mini wide의 공칭 깊이 범위 0.1–8 m | 범위 필터 0.1–8 m, 640×400·수직 FOV 60° 설계 pinhole |
| 힘/토크 | SDK `ft_sensor_left/right.force`, `.torque`는 N·Nm | 플랜지 로컬 프레임의 이상적 6축 FT |
| 관절·베이스 | SDK는 관절 position/velocity/torque, odometry와 갱신 시각을 제공 | 관절·휠 엔코더 에뮬레이션과 휠 오도메트리 |

D405는 USB, ZED X Mini는 GMSL2 연결을 사용한다. 실제 구성에서는 ZED에 맞는 호스트·캡처 인터페이스가 필요하다.

카메라 모델명은 대응할 하드웨어를 뜻한다. MuJoCo는 stereo 영상에서 깊이를 복원하지 않고 RGB와 metric depth를 렌더링한다. 제조사 SDK의 광학·반사·투명체 실패나 실측 정밀도를 재현한 결과가 아니다. 정사각 픽셀 pinhole의 수평 FOV는 렌더 종횡비로 결정되므로 제조사의 H/V 사양 전체와 동일하지 않다.

## 좌표계와 보정

- 월드 z는 바닥 위쪽이다. 초기 베이스와 목표 설비 좌표는 등록된 스테이션 설계값이다. SLAM이나 미지 환경 자율 탐색을 검증하지 않는다.
- optical은 x 오른쪽·y 아래·z 전방이다. 깊이는 m 단위 optical z이며 유클리드 사거리가 아니다.
- K는 렌더 해상도의 3×3 내부행렬이다. `fx=fy=H/(2 tan(fovy/2))`, `cx=(W-1)/2`, `cy=(H-1)/2`로 생성한다.
- `T_mount_optical`은 설정의 4×4 설계 변환이다. 머리는 `link_head_2`, 손목은 `link_left/right_arm_6`에 장착한다.
- `T_world_optical = T_world_base(encoder odometry) · FK(joint encoders) · T_mount_optical`이다. live free-base 정답 `cam_xpos`는 제어 패킷에서 제거한다.
- FT는 `left/right_ft_frame`, 손목 로컬 z=-0.1261 m의 플랜지 경계다. 힘 N·모멘트 Nm다. 실제 센서의 축·부호·모멘트 원점이 다르면 wrench adjoint 변환이 필요하다.
- 실제 장비의 해당 해상도 K·왜곡·rectification/alignment·hand–eye 외부행렬·로봇 영점과 기구 오차는 실측 파일로 교체해야 한다. `synthetic_model`을 `measured`로 이름만 바꾸면 보정으로 인정되지 않는다.

케이스는 공칭 외형의 box 근사다. D405의 0.060 kg와 box 관성·장착 TF는 합성 설계값이며 브래킷·배선 포함 실측값이 아니다. 머리 카메라 외형·질량도 선택한 버전과 액세서리에 맞춰 재확인이 필요하다.

## 데이터 형식과 주기

| 파일 | 내용 |
|---|---|
| `controller-data/schema.json` | 필드·단위·배열 차원·어댑터 경계 |
| `controller-data/frames.json` | 카메라명·NPZ 경로·관측 시각 |
| `controller-data/observations/*.npz` | RGB uint8 H×W×3, depth_m, 이상적 렌더 raw RGB/depth, K, T_world_optical, timestamp, metadata JSON |
| `controller-data/perception.jsonl` | 중심·yaw·형상 적합 품질·파지점·손목 확인 결과 |
| `controller-data/commands.jsonl` | 명령 시각·단계·몸통/양팔 20관절 위치/속도, 머리 2관절 기준값, 공통 그리퍼 슬라이드 관절 기준값, 네 휠 속도, 같은 tick의 몸통/팔/머리/휠 엔코더·그리퍼·FT 관측 |
| `config.json`·`upstream.json`·`manifest.json` | 설정·가정, 원본·소스 커밋·버전·산출물 해시 |
| `trace.jsonl` | 물체 정답 위치·접촉을 포함한 독립 평가 기록 |

제어용 RGB-D는 정차 후 인식·파지 검증 시점의 **단계 경계 관측**이다. 별도 `monitor_only` RGB-D는 1 Hz 공통 배치로 머리와 두 손목을 기록하고 6패널 표시용으로 사용한다. 모니터의 RGB·metric depth·K·TF·시각은 `monitor-observations/*.npz`에, 배치와 역할은 `monitor-frames.json`에 저장한다. 모니터를 30 FPS 제어 스트림으로 보고하지 않는다. 제어·관절·휠·힘 관측은 시뮬레이션 500 Hz, 요약 평가 trace는 50 Hz다. 실물 SDK 수신률·네트워크 지연을 측정한 값이 아니다.

`timestamp_sim_s`는 단조 시뮬레이션 시간이다. 실제 드라이버는 카메라 hardware clock과 로봇 system clock의 offset·drift를 보정하고 노출 시각의 엔코더로 TF를 보간해야 한다. RGB/depth 시각·해상도·정렬·좌표계가 맞지 않거나 오래된 관측은 거부해야 한다. 합성 `delivery_delay_s`는 노출 시각을 바꾸지 않고 물리 시간을 진행한 뒤 관측을 전달한다. 노출 시각의 K·TF를 보존하고, 미래·오래된 관측과 전달 timeout을 거부한다. `calibration_translation_bias_m`·`calibration_rotation_bias_rad`는 추정 외부행렬에 주는 시험 오차이며 실측 보정값이 아니다. 정차 이벤트의 합성 지연 검증을 움직이는 카메라의 지연 내성으로 확대 해석하지 않는다.

## 인식·파지 결과 해석

기본 인식은 깊이의 수평 높이 성분→연결 성분→바닥판 평면→CAD 직사각형 적합으로 중심과 yaw를 얻는다. 기존 색상 분할 방식은 비교군이다. 같은 위치의 바닥판·테두리 후보를 병합하고, 서로 다른 위치의 복수 CAD 가설은 거부한다. 이동·하강·접근 후 재관측의 위치·yaw 변화량을 검사한다. 손목은 예상 테두리의 국소 깊이 평면·높이·관측 폭을 검사하며 RGB 색상을 필수 조건으로 쓰지 않는다. 동일 위치와 형상의 가림 물체까지 한 장의 깊이 영상으로 식별할 수 있다는 주장은 하지 않는다. 화면에 잘린 관측과 치수가 맞지 않는 파란 방해물을 거부한다. 직사각형 yaw는 π 대칭 모호성을 갖는다. `covariance_xyzyaw`는 픽셀 크기·잔차 기반 `heuristic_uncertainty_proxy`이며 반복 실측으로 보정한 통계 공분산이 아니다. 보정 편향·깊이 스케일·시간 동기화 등의 체계 오차는 포함하지 않는다.

고정 upstream의 왼손·오른손 finger joint zero와 body offset은 대칭이 아니다. 공통 m 기준값은 두 손에 같은 개구 폭을 뜻하지 않는다. 양손의 실제 관절값을 따로 저장하며 실기에서는 각 손의 개구/전류 보정을 별도로 해야 한다. 그리퍼 CAD 충돌 교정은 [모델링 문서](modeling.md)에 명시한다.

그리퍼의 MuJoCo `actuator_force`는 시뮬레이션 힘 N이다. 실물 전류 A나 측정 파지력과 같지 않다. 전류→힘 변환에는 토크상수·감속비·효율·마찰·기구 보정이 필요하다. 네 손가락 접촉·대향 법선·적재물 보존·최종 안착은 별도의 정답 평가기가 판정한다.

## 실기 대응 시 남는 값

| 구현 경계 | 실측·드라이버에 필요한 값 |
|---|---|
| RGBDFrame·분리된 pose estimator | ZED/RealSense 정렬 영상·깊이·유효성·장치 시각·프로파일별 K/왜곡 |
| 휠 엔코더 odometry | 실제 휠 순서·부호·반경·축간 거리·slip·바닥 조건·위치 보정 |
| 이름 기반 20관절 위치/속도 | SDK model joint-order mapping·영점·방향·한계·전송률·추종 응답 |
| 슬라이드 관절 m·시뮬레이션 N | 실제 전류/엔코더↔개구 폭, 전류→힘 식별·백래시·패드 마찰 |
| 이상적 FT·정지 자세 tare | 실제 wrench frame·부호·영점·공구 질량/CoM 중력 보상·대역폭·분해능 |
| 합성 물성 | 트레이·적재물·카메라·그리퍼 질량/관성·접촉/servo 식별 |
| 동기 2 ms loop | 장비별 freshness·clock 변환·지연/dropout 재검증 |

실물 SDK 구동 드라이버는 포함하지 않는다. 제어 입력과 명령을 재생 가능한 데이터 계약으로 제공하는 범위이며 하드웨어 전이 완료를 뜻하지 않는다.

## 공식 자료

- [RB-Y1 제품·V1.2/V1.3 구성](https://www.rainbow-robotics.com/en/products/rb-y1)
- [Rainbow Robotics 비전 옵션 발표](https://www.linkedin.com/posts/rainbow-robotics_rainbowrobotics-rby1-visionsolutions-activity-7376893384017301504-JD_g)
- [Intel D405 공식 사양](https://www.intel.com/content/www/us/en/products/sku/229218/intel-realsense-depth-camera-d405/specifications.html)
- [Stereolabs 카메라별 깊이 범위](https://support.stereolabs.com/articles/9175329825-for-the-depth-sensing-what-is-the-range-for-the-zed-cameras-in-terms-of-measuring-distances-and-the-accuracy-of-the-detection)
- [RB-Y1 SDK RobotState 필드·단위·시각](https://rainbowrobotics.github.io/rby1-dev/sdk/cpp/state/robot-state.html)
- [RB-Y1 ROS 2 driver](https://rainbowrobotics.github.io/rby1-dev/ros2/ros2_driver.html)
