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

## 고정 행렬의 측정 결과

44회가 모두 실행됐고, 산출물·소스 해시와 500 Hz 제어 데이터·오프라인 인식 검증을 통과했다. 작업 성공과 검증 통과는 다른 의미다. 경로 비교의 결과는 다음과 같다.

| 조건 | 기존 완료 | 개선 완료 | 상승 중 접촉 표본: 기존 seed 0 / 1 | 개선 seed 0 / 1 |
|---|---:|---:|---:|---:|
| nominal | 2/2 | 2/2 | 0 / 0 | 0 / 0 |
| 깊이 잡음 3 mm·누락 3% | 2/2 | 2/2 | 0 / 0 | 0 / 0 |
| 적재물 질량 1.2배 | 2/2 | 2/2 | 0 / 0 | 0 / 0 |
| yaw +0.035 rad | 0/2 | 1/2 | 198 / 19 | 0 / 0 |
| yaw −0.035 rad | 0/2 | 2/2 | 172 / 251 | 0 / 0 |
| 미사용 검증 yaw +0.025 rad | 0/2 | 2/2 | 278 / 300 | 0 / 0 |
| 합계 | 6/12 | 11/12 | | |

같은 case·seed에서 양쪽 모두 완료한 6쌍의 완료시간은 개선 경로가 평균 0.101333 s 길었다. 개선 경로만 완료한 경우는 5쌍, 양쪽 모두 실패한 경우는 1쌍이고 기존 경로만 완료한 경우는 없었다. 보정 편향 +5 mm는 손목 검증 거부 2/2회, RGB-D 지연 150 ms는 인식 거부 2/2회로 기대 결과를 만족했다.

전체 판정은 `partial`이다. 기존 성공 조건을 보존하고 회전 조건의 접촉을 줄였지만, 핵심 yaw 조건의 전체 완료 gate는 통과하지 못했다. [실행별 결과와 영상](../evidence/improvement/README.md)을 함께 보존한다. 두 대표 영상은 각각 기존 seed 0의 벽 간섭과 개선 seed 1의 감속 시간 초과를 보여준다. 서로 다른 seed이므로 영상만으로 변경 전후 인과 비교를 하지 않는다.

## 개선 후 남은 협조 감속 실패

`tray-yaw-0.035`, seed 1의 개선 경로는 상승 구간의 기록 표본 3,243개에서 트레이와 벽 접촉이 없었지만, `drive_high`에서 150.932 s에 시간 초과로 끝났다. 적재물은 유지됐으며 안착 단계에는 도달하지 못했다. 같은 조건에서 협조 감속을 끈 실행은 148.356 s에 전체 과제를 완료했다.

협조 감시기는 첫 운반 표본의 좌우 하중 불균형을 기준으로 고정한다. 이 실행의 기준은 +0.6584였고, 이후 실제 불균형은 약 −0.0120으로 줄었다. 그러나 기준과의 차이를 검사하므로 균형에 가까워진 상태에서도 감속이 유지됐다. 높은 위치로 올리는 시간은 seed 0의 30.12 s에서 seed 1의 64.84 s로 늘었고, 높은 위치에서 전진할 때도 평균 속도 배율은 약 0.4515였다.

팔 궤적과 주행 명령에는 감속 배율이 적용되지만 `drive_high`의 20 s 제한은 물리 시각을 따른다. seed 1은 도착 기준까지 2.276 mm가 남고 약 4.967 mm/s로 움직이는 상태에서 종료됐다. 종료 시 양손 상대 위치 오차는 0.313 mm, 상대 자세 오차는 0.000732 rad로 감속 경고 기준보다 작았다. 이 실행에서는 FT 기준값의 의미와 제한 시간의 계산 방식이 충돌했다.

두 seed의 인식 yaw가 1°와 2°로 달랐고 트레이 기울기도 달랐지만, yaw 차이만으로 하중 분배 차이를 설명할 수 있는지는 분리 검증하지 않았다. 시간 초과 제한을 늘리거나 안전 판정 임계값을 완화해 이번 행렬의 결과를 바꾸지 않는다. 현재 결과에는 이 실패를 포함한다. 후속 제어 실험에서는 감속과 같은 누적 진행 시간으로 단계 예산을 계산하되, 별도의 절대 시간 상한과 협조 중단 조건을 유지하는 방식을 검증할 수 있다.

협조 비교 16회에서는 감속 사용 7/8회, 해제 8/8회가 작업을 완료했다. 아래 차이는 양쪽이 모두 완료한 7쌍의 평균이며 부호는 사용 − 해제다. 이번 조건에서 협조 감속의 작업 성공 우위는 확인되지 않았다.

| 지표 | 평균 차이 | 비교 범위 |
|---|---:|---|
| 작업 완료시간 | +1.178857 s | 완료한 전체 과제 |
| 최대 트레이 기울기 | −0.000141710 rad | `lift`부터 `seat`까지 |
| 최대 적재물 상대 변위 | +0.000636781 m | 같은 운반 구간 |
| 최대 양손 상대 위치 오차 | −0.000005328 m | 협조 감시 구간 |
| 최대 양손 상대 자세 오차 | −0.000005952 rad | 협조 감시 구간 |
| 최대 관절 구동기 토크 | 0 N·m | 전체 실행의 최대값 |

전체 실행의 최대 토크는 감속이 시작되기 전 단계에서 결정될 수도 있으므로, 차이가 0이라는 결과를 운반 중 토크가 항상 같다는 뜻으로 해석하지 않는다. 기울기·변위 차이는 작고 조건별 방향도 같지 않다. 이 작은 행렬로 일반적인 안정성 개선을 주장하지 않는다.

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
