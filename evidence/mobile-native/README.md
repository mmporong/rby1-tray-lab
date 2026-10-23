# 이동형 깊이 인식·양팔 운반 실행 근거

소스 커밋 `c7967a9aab6a31fafc574150d959c36477f9cf59`의 실행이다. 모델 기본 자세에서 이동 준비 자세로 전환해 접근·정차하고 트레이를 인식했다. 파지 자세로 낮춘 후와 파지 접근 후에 재관측하고, 손목 깊이 형상과 FT로 파지를 확인했다. 적재 주행·1.20 m 컨테이너 안착·손 회수 후 출발 위치까지 후진해 기본 자세로 복귀했다.

전체 작업은 151.232초에 6개 성공 조건을 통과했다. 주행·상승 중 양팔 상대 자세와 FT 지지력 비율을 50 Hz로 검사해 속도를 조절했다. 이는 기준 궤적의 감속과 지속 임계 초과 중단이며 힘 목표를 추종하는 제어는 아니다.

| 파일 | 역할 |
|---|---|
| [rollout.mp4](rollout.mp4), [rollout.gif](rollout.gif) | 같은 물리 실행의 약 4배속 렌더 |
| [initial.png](initial.png), [final.png](final.png) | 출발과 안착·후진·기본 자세 복귀 상태 |
| [result.json](result.json), [summary.json](summary.json) | 151.232초, 6개 조건 성공, 최대 토크·접촉·감속 지표 |
| [home-cycle-validation.json](home-cycle-validation.json) | 실제 시작·종료 상태, 후진 변위, 복귀 오차, 안착 유지 |
| [trace.jsonl.gz](trace.jsonl.gz) | 50 Hz 상태 기록. 각 물리 스텝의 독립 평가 결과 포함 |
| [controller-data/commands.jsonl.gz](controller-data/commands.jsonl.gz) | 500 Hz 명령·엔코더·그리퍼·FT 75,616개 |
| [controller-data/frames.json](controller-data/frames.json) | 정차 제어 관측 7프레임. 머리 3회, 양 손목 각각 2회 |
| [controller-data/monitor-frames.json](controller-data/monitor-frames.json) | 1 Hz 모니터 456프레임의 카메라·시각·역할 |
| [controller-data/monitor-observations](controller-data/monitor-observations) | 모니터 RGB·metric depth·K·TF·시각 NPZ |
| [controller-data/perception.jsonl](controller-data/perception.jsonl) | 깊이/CAD 추정, 재관측 변화량, 국소 테두리 검증, 안착 판정 |
| [controller-data/cooperation.jsonl](controller-data/cooperation.jsonl) | 운반 중 양팔 상대 자세·FT 하중 비율·감속 판단 |
| [controller-data/schema.json](controller-data/schema.json) | 단위·관절 순서·명령과 관측 경계·기본 자세 기준 |
| [controller-validation.json](controller-validation.json) | 패킷 무결성과 저장 관측 3회의 오프라인 인식 재실행 PASS |
| [sensor-preview.png](sensor-preview.png) | 물리 40.002초 공통 배치의 RGB·고정 범위 깊이 6패널 |
| [load-heatmap.png](load-heatmap.png), [SVG](load-heatmap.svg), [load-analysis.json](load-analysis.json) | 50 Hz 시간×관절 토크/모델 제한 비율, 손가락 접촉력과 수치·단위 |
| [load-view.png](load-view.png), [load-view.json](load-view.json) | 46초의 3D 관절 부하 표시와 별도 재실행 대조 근거 |
| [manifest.json](manifest.json) | 원본 소스·산출물 SHA-256. gzip은 압축 해제 내용으로 대조 |
| [presentation-manifest.json](presentation-manifest.json) | 표시용 파생 자료·재생 자료·GUI 검증 연결 |

파지 후 첫 안착 전 trace 3,559개가 모두 secure였고, 최소 손가락 접촉력은 2.534 N이었다. 최대 관절 추종 오차는 0.004848 rad, 최대 구동기 토크는 239.207 Nm다. 실제 복귀 최대 관절 오차는 0.000651 rad, 베이스 위치 오차는 10.859 mm다. 선언한 자세·속도 허용 오차에서 0.5초 유지해야 성공한다.

카메라 모니터는 세 대를 같은 물리 시각의 공통 배치로 기록한다. RGB와 깊이는 같은 NPZ에서 나오며 깊이는 미터 단위 optical Z다. 화면 PNG는 센서별 고정 범위를 사용하고 유효하지 않은 깊이를 자홍색으로 표시한다. 모니터 캡처는 복제한 MuJoCo 상태를 사용해 물리 solver cache를 바꾸지 않는다. 제어용 관측과 모니터 역할을 구분하며 둘을 30 FPS 실시간 제어 스트림으로 부르지 않는다.

`evidence/playback-native`는 같은 소스·설정의 별도 물리 재실행이다. 60 FPS·1,136프레임·18.933초 영상으로 표시 1배속이 시뮬레이션 8배속에 대응한다. 두 실행의 151.232초·6개 조건 성공을 대조했다. 실물 실행·보정·하드웨어 전이 성능은 검증 범위에 포함하지 않는다. [센서·전이 계약](../../docs/sensors-and-transfer.md)에 교체할 값과 인터페이스를 기록한다.

대표 실패 영상은 기존 두 개를 보존한다. 조건별 실험의 실패는 영상 수를 늘리지 않고 설정·결과·물리 기록으로 남긴다.

[조건별 비교 18회](../robustness/README.md)의 요약·설정·원본 manifest도 보존한다. 부하 히트맵은 강체의 관절 토크와 접촉력이며 구조 응력·파손이나 실물 안전성을 검증하지 않는다. 모델 토크 제한도 하드웨어 실측 정격이 아니다. 50 Hz 표본 최대 토크 239.030 Nm와 500 Hz 전체 스텝 최대값 239.207 Nm를 구분한다.

저장소 루트에서 검사한다.

```bash
.venv/bin/python tools/verify_evidence.py evidence/mobile-native
.venv/bin/python tools/verify_controller_data.py evidence/mobile-native
```
