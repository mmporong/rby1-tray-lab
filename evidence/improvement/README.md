# 회전 트레이 경로 개선 근거

**경로 비교 6/12 → 11/12회 완료, 최종 판정 `partial`.**

회전한 트레이의 림과 컨테이너 외벽 간섭을 재현하고, RGB-D 추정 yaw·등록된 CAD 외곽·벽 외부 면으로 상승 경로의 여유를 계산했다. 인식·파지·협조 감속·성공 판정 임계값은 바꾸지 않았다. 한 조건에서 협조 감속 시간 초과가 남아 전체 강건성 통과로 표시하지 않는다.

물리 소스는 `7c2ba29d355487d6b00e23c359e6f826c5959abc`의 깨끗한 checkout이다. 24회 경로 비교, 4회 센서 이상 거부, 16회 협조 감속 비교를 사전 등록한 행렬대로 실행했다. 중단된 이전 실행과 pilot는 이 결과에 합산하지 않았다. [설계·실패 진단·재현 방법](../../docs/improvement-study.md), [기계 판독 보고](report.json), [전체 번들 해시](manifest.json)를 함께 제공한다.

## 측정 결과

- 경로 비교: 기존 6/12회, 개선 11/12회 완료. 같은 case·seed에서 개선만 완료 5쌍, 둘 다 완료 6쌍, 둘 다 실패 1쌍, 기존만 완료 0쌍.
- 개선 경로 12회 모두 `raise_high`의 20 ms 기록 표본에서 트레이·벽 접촉 0. 이는 연속 500 Hz 무접촉 보증이 아니다.
- 양쪽 모두 완료한 경로 비교 6쌍에서 개선의 평균 완료시간 차이는 +0.101333 s. 실패 종료시각을 완료시간으로 계산하지 않는다.
- 보정 편향 +5 mm: 손목 검증 거부 2/2회. RGB-D 지연 150 ms: 인식 거부 2/2회. 기대 거부는 작업 성공과 구분한다.
- 협조 감속 사용 7/8회, 해제 8/8회 완료. 둘 다 완료한 7쌍의 평균 완료시간 차이(사용 − 해제)는 +1.178857 s. 안정성·성공률 우위는 확인되지 않았다.

조건당 seed 2개의 작은 행렬이다. 여기에 제시한 횟수는 일반 성공률이나 실물 성능 추정치가 아니다. 실물 토크·접촉력 실측, 실제 카메라 stereo 처리, 실물 전이, 학습 정책은 검증하지 않았다.

## 대표 실패 영상

| 영상 | 조건 | 원본 물리 결과 |
|---|---|---|
| [기존 경로 · 벽 간섭과 적재물 손실](videos/0.mp4) | yaw +0.035 rad, seed 0, 기존 경로 | 104.034 s, `payload_lost` |
| [개선 후 남은 협조 감속 실패](videos/1.mp4) | yaw +0.035 rad, seed 1, 개선 경로·감속 사용 | 150.932 s, `phase_timeout` |

두 영상은 서로 다른 seed의 실패 원인 예시다. 영상끼리 변경 전후 성능을 비교하지 않고 아래 전체 case·seed 행렬을 사용한다. 핵심 gate가 실패해 개선 성공 영상을 대표 영상으로 고르지 않았다. 기존 정상 전체 동작과 센서·응력은 [원본 재생 자료](../playback-native/manifest.json)에서 유지한다.

영상은 원본 설정·소스에서 새로 물리 실행하고 결과·시각·핵심 metric 일치를 검증했다. 화면 1배속은 물리 8배속, 60 FPS·960×720이며, [영상 0 manifest](videos/0.manifest.json)와 [영상 1 manifest](videos/1.manifest.json)에 설정·소스·재실행 결과를 보존한다. 영상 내부 픽셀이 실행을 독립 인증한다는 뜻은 아니다.

## 실행별 결과

완료는 파지·상승·안착·해제·회수·초기 자세 복귀의 기존 6개 조건을 모두 만족한 경우다. 표의 시각은 실패를 포함한 **종료시각**이며 완료시간 성능 비교용 평균이 아니다.

### 경로 비교 24회

| 조건 | 설정 | seed | 작업 결과 | 종료시각(s) |
|---|---|---:|---|---:|
| nominal | 기존·감속 사용 | 0 | [완료](improvement/runs/nominal__baseline-depth-cad__seed-0/result.json) | 151.232 |
| nominal | 개선·감속 사용 | 0 | [완료](improvement/runs/nominal__candidate-depth-cad__seed-0/result.json) | 151.440 |
| nominal | 기존·감속 사용 | 1 | [완료](improvement/runs/nominal__baseline-depth-cad__seed-1/result.json) | 150.728 |
| nominal | 개선·감속 사용 | 1 | [완료](improvement/runs/nominal__candidate-depth-cad__seed-1/result.json) | 151.226 |
| depth-noise-0.003-dropout-0.03 | 기존·감속 사용 | 0 | [완료](improvement/runs/depth-noise-0.003-dropout-0.03__baseline-depth-cad__seed-0/result.json) | 148.348 |
| depth-noise-0.003-dropout-0.03 | 개선·감속 사용 | 0 | [완료](improvement/runs/depth-noise-0.003-dropout-0.03__candidate-depth-cad__seed-0/result.json) | 148.354 |
| depth-noise-0.003-dropout-0.03 | 기존·감속 사용 | 1 | [완료](improvement/runs/depth-noise-0.003-dropout-0.03__baseline-depth-cad__seed-1/result.json) | 148.346 |
| depth-noise-0.003-dropout-0.03 | 개선·감속 사용 | 1 | [완료](improvement/runs/depth-noise-0.003-dropout-0.03__candidate-depth-cad__seed-1/result.json) | 148.356 |
| payload-mass-scale-1.2 | 기존·감속 사용 | 0 | [완료](improvement/runs/payload-mass-scale-1.2__baseline-depth-cad__seed-0/result.json) | 149.224 |
| payload-mass-scale-1.2 | 개선·감속 사용 | 0 | [완료](improvement/runs/payload-mass-scale-1.2__candidate-depth-cad__seed-0/result.json) | 149.154 |
| payload-mass-scale-1.2 | 기존·감속 사용 | 1 | [완료](improvement/runs/payload-mass-scale-1.2__baseline-depth-cad__seed-1/result.json) | 149.914 |
| payload-mass-scale-1.2 | 개선·감속 사용 | 1 | [완료](improvement/runs/payload-mass-scale-1.2__candidate-depth-cad__seed-1/result.json) | 149.870 |
| tray-yaw-0.035 | 기존·감속 사용 | 0 | [payload_lost](improvement/runs/tray-yaw-0.035__baseline-depth-cad__seed-0/result.json) | 104.034 |
| tray-yaw-0.035 | 개선·감속 사용 | 0 | [완료](improvement/runs/tray-yaw-0.035__candidate-depth-cad__seed-0/result.json) | 148.356 |
| tray-yaw-0.035 | 기존·감속 사용 | 1 | [cooperative_transport_fault](improvement/runs/tray-yaw-0.035__baseline-depth-cad__seed-1/result.json) | 103.000 |
| tray-yaw-0.035 | 개선·감속 사용 | 1 | [phase_timeout](improvement/runs/tray-yaw-0.035__candidate-depth-cad__seed-1/result.json) | 150.932 |
| tray-yaw-minus-0.035 | 기존·감속 사용 | 0 | [cooperative_transport_fault](improvement/runs/tray-yaw-minus-0.035__baseline-depth-cad__seed-0/result.json) | 90.560 |
| tray-yaw-minus-0.035 | 개선·감속 사용 | 0 | [완료](improvement/runs/tray-yaw-minus-0.035__candidate-depth-cad__seed-0/result.json) | 148.364 |
| tray-yaw-minus-0.035 | 기존·감속 사용 | 1 | [cooperative_transport_fault](improvement/runs/tray-yaw-minus-0.035__baseline-depth-cad__seed-1/result.json) | 100.580 |
| tray-yaw-minus-0.035 | 개선·감속 사용 | 1 | [완료](improvement/runs/tray-yaw-minus-0.035__candidate-depth-cad__seed-1/result.json) | 148.366 |
| tray-yaw-0.025-heldout | 기존·감속 사용 | 0 | [payload_lost](improvement/runs/tray-yaw-0.025-heldout__baseline-depth-cad__seed-0/result.json) | 98.246 |
| tray-yaw-0.025-heldout | 개선·감속 사용 | 0 | [완료](improvement/runs/tray-yaw-0.025-heldout__candidate-depth-cad__seed-0/result.json) | 149.812 |
| tray-yaw-0.025-heldout | 기존·감속 사용 | 1 | [payload_lost](improvement/runs/tray-yaw-0.025-heldout__baseline-depth-cad__seed-1/result.json) | 100.932 |
| tray-yaw-0.025-heldout | 개선·감속 사용 | 1 | [완료](improvement/runs/tray-yaw-0.025-heldout__candidate-depth-cad__seed-1/result.json) | 150.730 |

### 센서 이상 거부 4회

| 조건 | 설정 | seed | 작업 결과 | 종료시각(s) |
|---|---|---:|---|---:|
| calibration-bias-0.005 | 개선·감속 사용 | 0 | [wrist_verification_failed](safety/runs/calibration-bias-0.005__candidate-depth-cad__seed-0/result.json) | 44.696 |
| calibration-bias-0.005 | 개선·감속 사용 | 1 | [wrist_verification_failed](safety/runs/calibration-bias-0.005__candidate-depth-cad__seed-1/result.json) | 44.696 |
| rgbd-stale-0.15 | 개선·감속 사용 | 0 | [perception_failed](safety/runs/rgbd-stale-0.15__candidate-depth-cad__seed-0/result.json) | 18.546 |
| rgbd-stale-0.15 | 개선·감속 사용 | 1 | [perception_failed](safety/runs/rgbd-stale-0.15__candidate-depth-cad__seed-1/result.json) | 18.546 |

### 협조 감속 비교 16회

| 조건 | 설정 | seed | 작업 결과 | 종료시각(s) |
|---|---|---:|---|---:|
| nominal | 개선·감속 사용 | 0 | [완료](cooperation/runs/nominal__candidate-depth-cad__seed-0/result.json) | 151.440 |
| nominal | 개선·감속 해제 | 0 | [완료](cooperation/runs/nominal__candidate-no-coop-depth-cad__seed-0/result.json) | 148.354 |
| nominal | 개선·감속 사용 | 1 | [완료](cooperation/runs/nominal__candidate-depth-cad__seed-1/result.json) | 151.226 |
| nominal | 개선·감속 해제 | 1 | [완료](cooperation/runs/nominal__candidate-no-coop-depth-cad__seed-1/result.json) | 148.370 |
| payload-mass-scale-1.2 | 개선·감속 사용 | 0 | [완료](cooperation/runs/payload-mass-scale-1.2__candidate-depth-cad__seed-0/result.json) | 149.154 |
| payload-mass-scale-1.2 | 개선·감속 해제 | 0 | [완료](cooperation/runs/payload-mass-scale-1.2__candidate-no-coop-depth-cad__seed-0/result.json) | 148.354 |
| payload-mass-scale-1.2 | 개선·감속 사용 | 1 | [완료](cooperation/runs/payload-mass-scale-1.2__candidate-depth-cad__seed-1/result.json) | 149.870 |
| payload-mass-scale-1.2 | 개선·감속 해제 | 1 | [완료](cooperation/runs/payload-mass-scale-1.2__candidate-no-coop-depth-cad__seed-1/result.json) | 148.362 |
| tray-yaw-0.035 | 개선·감속 사용 | 0 | [완료](cooperation/runs/tray-yaw-0.035__candidate-depth-cad__seed-0/result.json) | 148.356 |
| tray-yaw-0.035 | 개선·감속 해제 | 0 | [완료](cooperation/runs/tray-yaw-0.035__candidate-no-coop-depth-cad__seed-0/result.json) | 148.354 |
| tray-yaw-0.035 | 개선·감속 사용 | 1 | [phase_timeout](cooperation/runs/tray-yaw-0.035__candidate-depth-cad__seed-1/result.json) | 150.932 |
| tray-yaw-0.035 | 개선·감속 해제 | 1 | [완료](cooperation/runs/tray-yaw-0.035__candidate-no-coop-depth-cad__seed-1/result.json) | 148.356 |
| tray-yaw-minus-0.035 | 개선·감속 사용 | 0 | [완료](cooperation/runs/tray-yaw-minus-0.035__candidate-depth-cad__seed-0/result.json) | 148.364 |
| tray-yaw-minus-0.035 | 개선·감속 해제 | 0 | [완료](cooperation/runs/tray-yaw-minus-0.035__candidate-no-coop-depth-cad__seed-0/result.json) | 148.364 |
| tray-yaw-minus-0.035 | 개선·감속 사용 | 1 | [완료](cooperation/runs/tray-yaw-minus-0.035__candidate-depth-cad__seed-1/result.json) | 148.366 |
| tray-yaw-minus-0.035 | 개선·감속 해제 | 1 | [완료](cooperation/runs/tray-yaw-minus-0.035__candidate-no-coop-depth-cad__seed-1/result.json) | 148.366 |

## 검증 범위와 원본 보존

[원본 검증 보고](raw-verification.json)는 44/44회 산출물·소스 해시, 제어 명령·센서 시각/좌표계/범위, 오프라인 RGB-D 인식을 확인한 기록이다. 작업 실패 run도 데이터 검증을 통과할 수 있다. 센서 이상으로 일찍 거부된 실행에는 모든 후속 카메라 관측을 요구하지 않았다.

생성기는 원본 trace 지표와 접촉 감사를 재계산하고 모든 run 설정을 사전 등록된 기본 설정에서 재구성했다. compact bundle에는 설정·결과·manifest·지표 검증·접촉 감사·영상만 포함한다. 전체 RGB-D·명령·trace 원본은 저장소 외부 실행 디렉터리에 보존한다. 이 배포물에서 실행하는 검증기는 보존된 파일의 해시와 바인딩·행렬·판정을 확인하며, 제외된 원본 전체 바이트를 다시 읽지는 않는다.

```bash
cd "$HOME/rby1-tray-lab"
.venv/bin/python tools/verify_improvement_evidence.py evidence/improvement
```

`verified: true`와 `assessment.status: partial`은 함께 나타나는 정상 결과다. 전자는 기록 정합성, 후자는 과제 개선 gate 판정이다. [소프트웨어·브라우저 검증](validation.json)에 검사 범위와 한계를 기록한다.
