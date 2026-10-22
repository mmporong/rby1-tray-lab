# 이동형 기본 CAD 그리퍼 왕복 실행 근거

소스 커밋 `4963881fe6dc27aa9f7ef741721ec0fdef038cd2`과 `configs/mobile-native.json`의 실행이다. 모델 기본 자세에서 출발해 낮은 작업 자세를 준비하고, RGB-D 바닥 트레이 인식·양팔 파지·적재 주행·1.20 m 컨테이너 안착·손 해제·회수를 수행했다. 이후 출발 위치로 후진하고 같은 모델 기본 자세로 돌아와 정지했다.

| 파일 | 역할 |
|---|---|
| [rollout.mp4](rollout.mp4) / [rollout.gif](rollout.gif) | 같은 물리 실행의 약 4배속 프레임. MP4는 GIF 프레임을 H.264로 변환 |
| [initial.png](initial.png) / [final.png](final.png) | 첫 렌더와 안착·후진·기본 자세 복귀 후 상태 |
| [home-cycle-validation.json](home-cycle-validation.json) | 모델 qpos0 기준, 시작·종료 실제 상태, 후진 거리, 복귀 오차와 안착 유지 |
| [pickup-approach.png](pickup-approach.png) | 별도 60 FPS 기록의 169번 프레임, 물리 22.534초 |
| [result.json](result.json) | 140.246초, 성공, 6개 milestone, 최고 트레이 중심 1.307726 m |
| [summary.json](summary.json) | 파지 후 첫 안착 전 50 Hz trace 3,420개 모두 secure, 최소 접촉력 2.652 N, 금지 접촉 기록 0 |
| [trace.jsonl.gz](trace.jsonl.gz) | 실제 로봇·물체·접촉·독립 평가 상태. 물체 정답과 베이스 정답 pose는 제어 입력에서 제외 |
| [controller-data/schema.json](controller-data/schema.json) | 단위·joint order·한계·클록·home_reference·어댑터 경계 |
| [controller-data/frames.json](controller-data/frames.json) | 머리·양손목 RGB-D 5프레임의 시각과 원본 NPZ 경로 |
| [controller-data/commands.jsonl.gz](controller-data/commands.jsonl.gz) | 500 Hz 명령·엔코더·그리퍼·FT 70,123개. 준비·후진·복귀 포함 |
| [controller-data/perception.jsonl](controller-data/perception.jsonl) | 머리 인식·손목 확인·FT 안착 판정 |
| [controller-validation.json](controller-validation.json) | 패킷 무결성과 저장된 RGB-D 인식 재실행 PASS |
| [sensor-preview.png](sensor-preview.png) | 머리 1회·닫기 전 양손목 RGB/depth 표시. 깊이는 프레임별 1–99 백분위로 정규화 |
| [pickup-path-comparison.json](pickup-path-comparison.json) | 픽업 접근·하강·폐쇄·유지 구간에 한정한 과거 기립 경유와 현재 낮은 경로의 저장 엔코더 FK 비교 |
| [config.json](config.json) / [scene.xml](scene.xml) / [upstream.json](upstream.json) | 실행 설정·장면·고정 원본과 메시·라이선스 provenance |
| [manifest.json](manifest.json) | 실행 소스와 원본 산출물 SHA-256. gzip은 압축 해제 내용으로 대조 |
| [presentation-manifest.json](presentation-manifest.json) | 압축 파일·파생 파일 해시와 물리·재생 manifest 참조 |

모델 XML에는 별도 home 키프레임이 없다. `qpos0`의 몸통·양팔 20관절·헤드 2축은 0 rad, 그리퍼는 0 m다. 베이스 목표는 초기 스테이션 `[-0.4, 0, 0]`이다. 실제 종료 오차는 최대 관절 0.000636 rad, 헤드 0.000149 rad, 그리퍼 0.558 mm, 베이스 10.969 mm였다. 자세·속도가 선언한 허용 오차 안에서 0.5초 유지돼야 성공하며, 손 회수만으로는 실행이 끝나지 않는다. 베이스 정답 pose는 독립 평가 전용이다.

FT 사전 상향 지지는 4.806 N, 안착 후 상향 지지는 0 N, signed world-Z는 -12.784 N이었다. 관절 FK로 추론한 안착 높이 오차는 3.099 mm다. 양손 회수 이후 후진·자세 복귀 중에도 트레이와 세 적재물의 안착·지지·정지 상태를 2 ms마다 검사했다.

렌더 요청은 0.5초 간격이며 마지막은 140.246초다. GIF 재생 길이는 33.720초이며 정확한 물리 시각은 trace/commands를 따른다. `evidence/playback-native`는 같은 소스·설정의 별도 60 FPS 물리 재실행이다. 두 실행의 140.246초·6개 조건 성공을 대조했으며 한 프로세스의 기록이라고 주장하지 않는다.

센서·물성·보정은 합성 조건이다. 제조사 stereo 복원·실측 보정·실물 SDK 실행·하드웨어 전이 성공은 검증하지 않았다. [센서·전이 계약](../../docs/sensors-and-transfer.md)에 교체할 값과 인터페이스를 기록한다. 대표 실패 영상은 기존 두 개를 유지한다.

저장소 루트에서 검사한다.

```bash
.venv/bin/python tools/verify_evidence.py evidence/mobile-native
.venv/bin/python tools/verify_controller_data.py evidence/mobile-native
```

로봇 메시 복원은 `fetch-assets`를 따른다. `assets` 링크는 로컬 캐시 연결이며 Git에는 넣지 않는다.
