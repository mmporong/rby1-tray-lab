# 이동형 기본 CAD 그리퍼 실행 근거

소스 커밋 `9f9fd5590b267b63d268f17ddf95f6d8b0507de8`에서 `configs/mobile-native.json`으로 수행했다. 바닥 트레이 인식·낮은 자세의 양팔 파지·적재 주행·높은 컨테이너 안착·손 해제·회수를 하나의 MuJoCo 물리 실행으로 검증한다.

| 파일 | 역할 |
|---|---|
| [rollout.mp4](rollout.mp4) / [rollout.gif](rollout.gif) | 같은 실행의 렌더 프레임, 약 4배속. MP4는 GIF 프레임을 H.264로 변환한 보기용 파일 |
| [initial.png](initial.png) / [final.png](final.png) | 바닥 초기 상태와 안착·회수 후 상태 |
| [pickup-approach.png](pickup-approach.png) | 같은 소스의 별도 60 FPS 실행에서 추출한 파지 시점 105번 프레임, 물리 시각 14.000초 |
| [sensor-preview.png](sensor-preview.png) | 머리 1회·닫기 전 양손목 2회의 저장된 RGB/depth. 표시 깊이는 프레임별 1–99 백분위 구간으로 대비를 조정하고 범위를 표기 |
| [result.json](result.json) | 102.470초, 성공, 5개 milestone, 최고 높이 1.307512 m, 추종/토크 및 센서 안착 판정 |
| [summary.json](summary.json) | 파지 후 첫 안착 전의 50 Hz trace 요약. 3,419개 모두 secure, 최소 접촉력 2.635 N, 금지 접촉 기록 0 |
| [trace.jsonl.gz](trace.jsonl.gz) | 물체 정답 위치·접촉·관절·독립 평가 요약. 제어 입력으로 쓰는 데이터가 아님 |
| [controller-data/schema.json](controller-data/schema.json) | 단위·joint order·한계·클록·어댑터 경계 |
| [controller-data/frames.json](controller-data/frames.json) | 3대 카메라 5프레임의 시각·NPZ 경로. 닫기 전 2장과 파지 후 확인 2장을 모두 보존 |
| [controller-data/commands.jsonl.gz](controller-data/commands.jsonl.gz) | 같은 tick의 명령·엔코더·그리퍼·FT 51,235개. 머리 2·몸통/팔 20관절, 휠 4개 |
| [controller-data/perception.jsonl](controller-data/perception.jsonl) | 트레이 추정, 손목 확인, 안착 전 하중 전달 판정 |
| [controller-validation.json](controller-validation.json) | 패킷 무결성과 저장된 머리 RGB-D 인식의 오프라인 재실행 결과 |
| [config.json](config.json) / [scene.xml](scene.xml) / [upstream.json](upstream.json) | 실행 설정, 이동형 장면, 고정 원본·메시·라이선스 provenance |
| [manifest.json](manifest.json) | 실행 당시 Git 소스·원본 산출물 SHA-256. gzip은 압축을 푼 내용으로 대조 |
| [pickup-path-comparison.json](pickup-path-comparison.json) | 기립 경유 제거 전후의 저장 엔코더 FK, 파지 전 머리 높이 비교와 별도 두 실행의 성공 기록 |
| [presentation-manifest.json](presentation-manifest.json) | 압축 파일과 보기용 파생 파일의 SHA-256, 원본 manifest 참조 |

FT 기반 안착 판정에서 사전 상향 지지 하중은 4.759 N이었다. 안착 후 상향 지지는 0 N, signed world-Z는 -12.309 N으로 하향 내력이 남았으며 두 값을 별도로 기록했다. 관절 FK로 추론한 안착 높이 오차는 3.615 mm다. 독립 평가기는 실제 컨테이너 바닥 접촉·적재물 지지·로봇 접촉 해제·80 mm 회수·0.5초 유지를 따로 검사한다.

렌더는 0.5초 단위 요청이며 마지막 요청은 102.470초에서 조기 종료된다. GIF는 동일한 정지 화면을 병합하므로 전체 재생 길이는 24.6초이며 정확한 시간축은 trace/commands의 시각을 따른다.

센서와 물성은 합성 보정 조건이다. 제조사 stereo 깊이 복원·실측 카메라/로봇 보정·드라이버 지연·실물 성공을 검증한 결과가 아니다. [센서·전이 계약](../../docs/sensors-and-transfer.md)에 실제 장비에서 교체해야 할 값과 인터페이스를 명시한다. 이 단일 실행을 성공률이나 다양한 조건에 대한 강건성으로 해석하지 않는다.

저장소 루트에서 다음 명령으로 검사한다.

```bash
.venv/bin/python tools/verify_evidence.py evidence/mobile-native
.venv/bin/python tools/verify_controller_data.py evidence/mobile-native
```

장면의 로봇 메시를 다시 불러오려면 저장소 실행 절차의 `fetch-assets`로 고정 원본을 확보한다. `assets` 심볼릭 링크는 로컬 캐시 연결이며 Git에 포함하지 않는다.

`evidence/playback-native`는 같은 소스·설정으로 별도 재실행해 만든 60 FPS 기록이다. 두 실행 모두 102.470초에 5개 조건을 통과했다. 데이터와 영상이 하나의 프로세스에서 기록됐다는 뜻은 아니다.
