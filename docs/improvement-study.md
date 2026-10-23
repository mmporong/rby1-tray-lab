# 회전 트레이 운반 경로 개선 검증

이 연구는 RGB-D로 찾은 트레이를 양팔로 잡아 주행·상승·안착하는 과정에서 발견한 충돌 실패를 수정한다. 로봇 SW 포트폴리오의 검토 대상은 실패 원인, 제어와 평가의 경계, 같은 조건의 변경 전후 결과, 재현 가능한 기록이다. 학습 정책이나 실물 전이 성공을 주장하지 않는다.

## 문제와 가설

기존 `tray-yaw-0.035`, seed 0 기록에서는 트레이가 약 2° 회전한 채 상승하다 컨테이너 근측 벽에 걸리고, 이후 적재물을 잃었다. 원본 trace를 장면의 box collision geometry로 재구성했을 때 처음 관측된 접촉은 87.16 s, 최저 접촉 거리는 약 −0.687 mm다. 이후 트레이 pitch가 증가하고 104.034 s에 `payload_lost`로 끝났다. 베이스 tilt와 양손 상대 자세 오차가 커지기 전에 벽 접촉이 나타났다.

기존 상승 경로는 컨테이너 내부 앞쪽 경계 0.95 m에서 트레이 바닥 반폭 0.18 m와 여유 0.03 m만 빼, 트레이 중심을 x=0.740 m로 후퇴시켰다. 그러나 실제 벽의 외부 면은 x=0.926 m이고, 림을 포함한 트레이 +X 외곽은 yaw=0일 때 0.186 m, yaw=0.035 rad일 때 약 0.195334 m다. 기존 경로는 회전된 트레이 외곽을 벽 안으로 약 9.3 mm 밀어 넣을 수 있다.

가설은 **관측된 yaw로 회전한 림 외곽과 등록된 벽 외부 면을 사용하면 이 간섭을 제거할 수 있다**는 것이다. 인식·파지·협조 감속·성공 판정의 임계값은 유지한다.

## 변경 방법과 관측 경계

`transport_clearance.py`는 컴파일된 모델의 바닥·림 5개 box와 고정된 컨테이너 벽 4개 box를 읽는다. 지원하지 않는 geom 종류·회전·부모 body는 거부한다. 각 box의 회전된 중심과 반폭으로 +X support를 계산한다.

```text
safe_x = min(start_x, near_wall_outer_x − rotated_tray_positive_x_support − 0.03 m)
```

yaw=±0.035 rad에서 계산된 safe_x는 약 0.700666 m다. 실제 실행에서는 시뮬레이터의 정답 yaw 대신 **RGB-D 추정 yaw**를 넣으므로 실행별 수치는 `ik.json`을 확인해야 한다.

- 제어 경로: RGB-D 추정, 휠 오도메트리, 관절 엔코더, 손목 FT와 등록된 CAD/환경 지도.
- 사전검사: `raise_high`의 명령된 트레이 중심 경로를 표본화하고 벽과 수직으로 겹치는 구간에서 30 mm 여유를 요구한다. 관절 보간 검사에서도 같은 명령 중심을 검사한다.
- 독립 평가: 실제 물리 trace의 tray pose로 접촉을 재구성한다. 이 데이터는 제어기로 되돌리지 않는다.
- 의도된 `seat` 접촉은 운반 중 벽 접촉 금지와 구분한다.

30 mm는 명령 중심·추정 yaw에 대한 설계 여유다. 실제 FK, roll/pitch, 접촉 순응, 캘리브레이션 오차를 포함하는 하드웨어 안전 여유를 보증하지 않는다. 실물에서는 컨테이너 지도와 카메라 외부 보정을 확보해야 한다.

## 고정 비교 행렬

소스 커밋은 `7c2ba29d355487d6b00e23c359e6f826c5959abc`다. 별도의 깨끗한 checkout에서 동일한 Python·MuJoCo 환경으로 실행한다. 각 연구는 실행 전에 전체 행렬과 설정·소스 SHA를 저장한다.

| 연구 | 조건 | 조합 | 실행 수 |
|---|---|---|---:|
| 경로 개선 | nominal, 깊이 잡음 3 mm·누락 3%, 질량 1.2배, yaw ±0.035, 미사용 검증 yaw +0.025 rad | seed 0·1 × 기존/개선, 협조 감속 사용 | 24 |
| 안전 거부 | 카메라 외부 보정 +5 mm, RGB-D 지연 150 ms | seed 0·1, 개선 경로 | 4 |
| 협조 비용 | nominal, 질량 1.2배, yaw ±0.035 rad | seed 0·1 × 감속 사용/해제, 개선 경로 | 16 |

`+0.025 rad`는 구현 전에 고정한 검증 조건이며 경로 튜닝에 사용하지 않는다. 안전 거부는 작업 성공과 별도로 센서 거부 이유까지 확인한다. 보정 편향을 숨기는 오프셋 보상이나 gate 완화는 하지 않는다.

`configs/mobile-native-improved.json`은 단독 실행용 프리셋이다. benchmark는 동일한 동작 필드만 주입하므로 각 run의 `config.json`이 해당 실험의 권위 설정이며, 프리셋 파일의 설명 필드까지 같은 SHA를 가진다는 뜻은 아니다.

## 측정과 해석

- 완료 여부는 기존 독립 평가의 파지·상승·안착·해제·회수·초기 자세 복귀 6개 조건으로 판정한다.
- 완료시간 차이는 **양쪽 모두 성공한 같은 case·seed**에서만 계산한다. 실패까지 걸린 시간은 별도의 종료시각이며 속도 성능으로 해석하지 않는다.
- 트레이 roll/pitch/tilt와 적재물 변위는 `lift`부터 `seat`까지의 기록 표본에서 구한다. 적재물 변위는 첫 유효 `grasp_hold` 표본의 트레이 좌표계를 기준으로 하고, 없을 때의 fallback과 기준 시각을 기록한다.
- 최대 토크는 모델 구동기의 토크다. 실물 전류 기반 실측 토크나 하드웨어 연속 정격이 아니다.
- 협조 사용/해제는 동일 조건의 양쪽 성공 쌍에서 시간·기울기·변위·양손 상대 자세 오차·토크의 차이를 구한다. 차이의 부호는 **사용 − 해제**다.
- 실패와 성공 실행의 최대값은 종료 범위가 서로 다를 수 있다. 이를 같은 작업을 완수한 성능 차이로 합치지 않는다.
- `transport-audit.json`은 `raise_high`의 **20 ms 기록 표본**을 재구성한 결과다. 표본 사이를 포함한 500 Hz 연속 무접촉 증거가 아니다.
- 조건당 seed가 2개인 작은 행렬이다. 성공 수와 Wilson 구간을 일반 환경의 성공 확률로 확장하지 않는다.

## 재현

실행 중 소스를 수정하지 않는다. 출력 경로는 존재하지 않는 새 경로여야 한다. 전체 원본은 임시 디스크나 별도 실험 볼륨에 저장하고, JSONL은 지표 추출 후 gzip한다.

아래 명령은 현재 checkout으로 새 연구를 생성한다. 게시된 물리 실행과 같은 소스를 쓰려면 `git worktree add --detach "$HOME/rby1-tray-reproduce" 7c2ba29d355487d6b00e23c359e6f826c5959abc`로 별도 checkout을 만들고 그 위치에서 실행한다. 기존 가상환경을 공유할 때는 `PYTHONPATH="$HOME/rby1-tray-reproduce/src"`를 지정한다. 렌더링도 원본 실행과 같은 clean commit에서 수행해야 한다.

```bash
cd "$HOME/rby1-tray-lab"
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl EGL_PLATFORM=surfaceless
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
.venv/bin/python tools/benchmark_mobile.py --study improvement --compress-jsonl --output artifacts/study/improvement
.venv/bin/python tools/benchmark_mobile.py --study safety --compress-jsonl --output artifacts/study/safety
.venv/bin/python tools/benchmark_mobile.py --study cooperation --compress-jsonl --output artifacts/study/cooperation
```

실행별로 다음 명령을 사용한다. 경로 끝의 run ID는 해당 `preregistered.json`에서 확인한다.

```bash
.venv/bin/python tools/audit_transport_contacts.py artifacts/study/improvement/tray-yaw-0.035__candidate-depth-cad__seed-0
.venv/bin/python tools/verify_evidence.py artifacts/study/improvement/tray-yaw-0.035__candidate-depth-cad__seed-0
.venv/bin/python tools/verify_controller_data.py artifacts/study/improvement/tray-yaw-0.035__candidate-depth-cad__seed-0
```

대표 영상은 원본 run의 설정으로 같은 소스에서 새로 물리 실행한다. 성공/실패 원인·시각·핵심 metric이 원본과 허용오차 내에서 일치해야 MP4를 발행한다. 원본 trace의 누락 프레임을 합성한 영상이 아니다. 표시 1배속은 물리 8배속이며 60 FPS다.

```bash
.venv/bin/python tools/render_study_video.py artifacts/study/improvement/tray-yaw-0.035__candidate-depth-cad__seed-0 --output artifacts/study-video --case tray-yaw-0.035 --profile candidate-depth-cad
```

`evidence/improvement`의 compact bundle에는 기본 설정·사전 등록·요약·run 설정·결과·manifest·지표 검증·접촉 감사·대표 영상과 렌더 manifest를 보존한다. 생성기는 전체 설정을 재구성하고 원본 result·trace 지표·접촉 감사를 재계산한다. 영상은 실제 MP4 구조와 고정된 8배속·60 FPS 계약까지 확인한다. 전체 RGB-D/명령/trace는 원본 실행 폴더에 남고, 배포된 compact 검증은 보존된 파일과 바인딩을 검사한다. 원본 전체 바이트 재검증이나 독립적인 영상 실행 인증을 뜻하지 않는다.

`--videos` 파일에는 `source_manifest` 경로와 표시용 `label`만 넣는다. 설정·결과·배율은 렌더 manifest에서 읽는다. compact 생성은 새 도구가 있는 현재 checkout에서 수행하며, `--base-config`는 원본 연구에 사용한 설정을 지정한다.

```bash
.venv/bin/python tools/build_improvement_evidence.py artifacts/study evidence/improvement --base-config configs/mobile-native.json --videos artifacts/study-videos.json
```

```bash
.venv/bin/python tools/verify_improvement_evidence.py evidence/improvement
```

기존 상단 원본 영상·RGB-D·구조 응력은 과거 실행의 같은 타임라인을 유지한다. 개선 연구 영상은 하단의 별도 재생기이며 기존 센서·응력 프레임을 움직이지 않는다.
