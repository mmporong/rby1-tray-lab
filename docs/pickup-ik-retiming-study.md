# 파지 접근 IK 회전 감소 및 시간 배분 실험 (2026-11-17)

> 상태: **사용자 로컬 MuJoCo 3.11.0 / seed=0 실험에서 성공한 후보의 선택형 코드화**.
> 기본 실행 설정은 변경하지 않았다. 새 본체 구현의 전체 회귀 시뮬레이션과 다른 seed/장면에 대한 검증은 별도로 필요하다.
> 아래 수치는 기존 독립 실험 스크립트의 사용자 로컬 측정 출력이며, 이 커밋 자체에서 재측정한 값이 아니다.

## 문제와 원인

RB-Y1-M의 `pickup_approach`는 TCP 목표를 정확히 만족했지만, 마지막 IK 경유점에서
`HIGH_CARRY_WARM_START_RAD`를 초기 추정값으로 사용하면서 서로 다른 관절 구성이 연결되었다.
기존 실험의 구간별 최대 관절 변화량은 81.36° / **140.63°**였다.
접근 단계는 운반 중 관절 경유점 간 변화 제한(0.22 rad)의 적용 대상에서 제외되어 있다.
큰 회전이 곧 충돌이나 전기 소비량 증가를 의미하지는 않으므로 실제 물리 데이터를 비교했다.

## 비교 설계

- 원본: 기존 IK와 10초 시간 프로파일.
- 회전 감소: 마지막 IK 초기 추정값을 `(1-alpha)·이전 경유점 + alpha·고정 고자세 seed`로 혼합 (`alpha=0.75`); 동일 10초.
- 시간 최적화: 위 **동일한 IK 경로**의 첫 5초 프로파일을 그대로 유지하고 두 번째 IK 구간의 시간만
  `clamp(5초 × 후보 2구간 최대 변화량 / 원본 2구간 최대 변화량 × 계수, 1.5초, 5초)`로 조정.
  첫 구간/둘째 구간 접점은 C1(위치·속도 연속) Hermite 시간 보간.
- 로봇 모델, seed, 설정, 토크·충돌·TCP·그립·안착·복귀 합격 기준은 변경하지 않음.
- 저속 시간: 14개 팔 관절의 **모든 실제 속도**가 1°/s 이하인 물리 스텝의 합계.
- 단순한 조기 종료는 시간 단축으로 계산하지 않고, 6개 완료 조건을 모두 통과한 결과만 비교.

## 사용자 로컬 실험 결과

| 조건 | 전체 성공 | 전체 시간(s) | 접근 시간(s) | 저속 시간(s) | 2구간 저속(s) | 양팔 누적 이동(°) | 접근 양의 기계적 일(J) | 접근 최대 토크(N·m) |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| baseline_fixed | 예 | 150.840 | 10.148 | 2.460 | 1.234 | 1323.774 | 24.660 | 12.009 |
| reduced_fixed (alpha=0.75) | 예 | 153.890 | 10.148 | 2.570 | 1.294 | 1129.629 | 22.337 | 13.230 |
| reduced_timed_0.70 | 예 | **146.902** | **7.876** | **1.100** | **0.364** | **1129.635** | **23.069** | **13.038** |
| reduced_timed_0.85 | 예 | 161.640 | 8.460 | 1.180 | 0.444 | 1129.631 | 22.459 | 13.078 |
| reduced_timed_1.00 | 예 | 155.188 | 9.044 | 1.274 | 0.538 | 1129.629 | 22.060 | 13.188 |

`reduced_timed_0.70`은 원본 대비 전체 **3.938초(2.6%) 단축**, 양팔 누적 이동 **14.7% 감소**,
접근 양의 기계적 일 **6.5% 감소**를 보였다. 대신 접근 최대 토크는 **8.6% 증가**했다.
다른 시간 계수는 접근 시간은 짧아도 이후 `drive_high` 등 단계의 영향으로 전체 완료가 더 늦었다.
실험 결과는 비단조적이므로 0.70을 전역 최적값이라고 해석하지 않는다.

기존 단축 없는 원본과 `alpha=0.75`의 전체 3.050초 차이는 접근 단계가 아니라 주로
`drive_high`의 **+3.124초**에서 발생했다. 원본과 대안의 시간 배분을 같게 둔 비교에서는
접근 단계 시간 차이가 0초였다.

## 안전성·물리 해석의 경계

- 원본 / 회전 감소 고정시간 경로 모두 표본 관통 접촉 0스텝; 최소 표본 거리 0m.
  0m는 카메라 케이스와 지지대 다리 조합에서 발생하며, **안전 여유가 증가한 근거가 아니다**.
- 원본 / 기존 회전 감소 경로의 구동기별 최대 한계 사용률은 약 20.29%로 보고되었다.
  이는 각 구동기 한계에 대한 최댓값이며, 해당 실행의 최대 토크 수치와 동일한 관절이라는 뜻은 아니다.
- 양의 기계적 일은 MuJoCo의 토크·각속도 기반 지표다. 전기 소비량 또는 사고 확률이 아니다.
- 이 결과는 seed 0의 1회 실행씩 비교한 값이다. 다른 seed/트레이 yaw/센서 이상 조건에 대한 성공률을 주장하지 않는다.

## 본체 코드 선택적 사용

새 코드는 `pickup_optimization.enabled`가 `true`일 때만 작동한다.
기본 `configs/mobile-native.json`은 변경하지 않았다.

`configs/mobile-native-pickup-optimized.json`은 검증에 사용된 설정의 복사본에 다음을 더했다.

```json
"pickup_optimization": {
  "enabled": true,
  "seed_blend_alpha": 0.75,
  "baseline_second_jump_rad": 2.454371637500114,
  "second_segment_factor": 0.70,
  "minimum_second_s": 1.5
}
```

`baseline_second_jump_rad`는 **해당 기준 실행에서 측정된 경유점 차이**다.
다른 모델이나 설정에서 이 수치를 무조건 재사용하지 말고 새 원본 기록으로 다시 보정한다.

### 재현 명령 (프로젝트 루트, 기존 Python/MuJoCo 3.11.0 환경)

```bash
.venv/bin/python -m unittest discover -s tests -p 'test_pickup_timing.py'
.venv/bin/python -m unittest discover -s tests

MUJOCO_GL=egl PYOPENGL_PLATFORM=egl EGL_PLATFORM=surfaceless \
  .venv/bin/python -m rby1_tray.cli run \
  --config configs/mobile-native.json \
  --output artifacts/pickup-baseline-new

MUJOCO_GL=egl PYOPENGL_PLATFORM=egl EGL_PLATFORM=surfaceless \
  .venv/bin/python -m rby1_tray.cli run \
  --config configs/mobile-native-pickup-optimized.json \
  --output artifacts/pickup-optimized-new
```

각 `--output` 폴더가 없어야 하며 기존 결과를 덮어쓰지 않는다.
`result.json`의 `success`, `milestones`, `simulation_time_s`와
`ik.json`의 `pickup_time_schedule`을 확인한다. 작업 성공과 시간의 재현은 여기서 점검한다.
기계적 일/저속 구간/거리 분석은 동일 계측 정의의 독립 로컬 A/B 스크립트로 추가 검증해야 한다.

**기본값 승격 조건:** 복수 seed·yaw·노이즈 조건의 전체 성공,
기존 안전 판정 유지, 이동량/시간/기계적 일/토크 지표 비교를 모두 확인한 뒤 검토한다.
