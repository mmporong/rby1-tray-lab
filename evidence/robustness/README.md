# 이동형 인식·운반 조건별 비교

물리 소스 `c7967a9aab6a31fafc574150d959c36477f9cf59`에서 사전 등록한 조건으로 18회 실행했다. 현재 M형·원본 CAD 그리퍼 접촉 교정·무손잡이 트레이만 포함한다. 과거 A형 지그·손잡이 실험은 합산하지 않는다.

전체 작업 완료는 12회, 미완료는 6회다. 미완료 중 1회는 오래된 RGB-D 관측을 거부한 기대 동작이다. 대부분 조건과 방법 조합당 1회이며, 기본 조건의 depth/CAD 감속 유무만 seed 0·1을 포함한다. 이 집계를 일반 성공률이나 방법의 통계적 우위로 해석하지 않는다.

| 시험 조건 | 방법·표본 | 결과 |
|---|---|---|
| 기본, seed 0·1 | depth/CAD, 감속 유·무 각 2회 | 4회 모두 완료 |
| 회색 트레이, seed 0 | depth/CAD, 감속 유·무 각 1회 | 2회 모두 완료 |
| 깊이 노이즈 3 mm·누락 3%, seed 0 | depth/CAD, 감속 유·무 각 1회 | 2회 모두 완료 |
| 목표 지지면 1.10 m | depth/CAD·감속, 1회 | 완료, 151.138 s |
| 목표 지지면 1.25 m | depth/CAD·감속, 1회 | IK 거부, 66.158 s. 방향 잔차 0.0265 rad > 0.02 rad |
| 적재물 질량 ×1.2 | depth/CAD·감속, 1회 | 완료, 149.224 s |
| RGB-D 전달 지연 50 ms | depth/CAD·감속, 1회 | 완료, 151.126 s |
| RGB-D 전달 지연 150 ms | depth/CAD·감속, 1회 | 관측 나이 0.102 s에서 거부. 허용값 0.1 s |
| 광학 마운트 병진 보정 편향 5 mm | depth/CAD·감속, 1회 | 파지 확인 실패, 44.696 s |
| 트레이 마찰 ×0.7 | depth/CAD·감속, 1회 | 완료, 150.242 s |
| 트레이 초기 yaw 0.035 rad | depth/CAD·감속, 1회 | 적재물 이탈, 104.034 s |
| 회색 트레이 | 기존 blue RGB-D·감속, 1회 | 인식 실패, 18.444 s |
| 기본 파란 트레이 | 기존 blue RGB-D·감속, 1회 | 적재물 이탈, 100.128 s |

감속은 기본 seed 0에서 작업 시간을 148.352 s에서 151.232 s로 늘렸다. 두 방법 모두 성공했으며 성공률 우위는 입증하지 않았다. yaw 변화 조건에서는 감속이 적용돼도 적재물 이탈을 막지 못했다. 기존 blue RGB-D는 기본 조건에서도 완주하지 않았으므로 회색 실패만으로 두 방법의 차이를 설명하지 않는다. 실제 인식·파지·운반의 결과를 함께 기록한다.

1.25 m 결과는 현재 경로·자세·IK 설정의 거부이며 로봇의 전역 도달 높이 한계가 아니다. 보정 편향도 시험한 축·크기 한 조건의 결과이며 실물 보정 허용 오차를 뜻하지 않는다. 질량 배율은 트레이 위 적재물에만 적용한다.

## 저장 범위

- [summary.json](summary.json): 18회 전체 결과, 설정 해시, 실패 분류, 관절 토크·협조 감속 요약.
- 각 그룹의 `preregistered.json`, `summary.json`, `dataset-manifest.json`: 실행 전 행렬과 결과, 원본 run manifest 해시.
- 각 실행의 `config.json`, `result.json`, `manifest.json`, `perception.jsonl`: 원본을 바꾸지 않고 복사한 설정·판정·인식 기록.
- [compact-manifest.json](compact-manifest.json): 이 배포 묶음에 실제 포함한 파일의 SHA-256.

**이 폴더는 전체 원시 실행을 포함하지 않는 배포용 요약 묶음이다.** 각 run의 원본 `manifest.json`에는 대용량 trace·500 Hz 제어 패킷·RGB-D NPZ도 선언돼 있으나 이 폴더에는 포함하지 않는다. 원본 전체 실행은 작업 기기의 `artifacts/upgrade-*-c7967a9`에 보존하고 `tools/verify_evidence.py`로 해시를 검사했다. 압축한 trace와 명령은 압축 해제 내용의 원본 SHA를 대조했다. 완전한 기본 실행 데이터는 [mobile-native](../mobile-native/README.md)에 별도로 포함한다.

## 재현

저장소 루트에서 실행한다. 각 명령의 output은 아직 존재하지 않는 경로를 사용한다. 당시 소스가 필요하면 별도 checkout에서 위 물리 소스를 선택한다. seed·조건·방법과 합성 센서 가정은 각 그룹의 사전 등록 및 설정 파일이 기준이다.

```bash
.venv/bin/python tools/benchmark_mobile.py --output artifacts/reproduce-comparison --cases nominal,gray-tray,depth-noise-0.003-dropout-0.03 --seeds 0 --variants coop-depth-cad,no-coop-depth-cad
.venv/bin/python tools/benchmark_mobile.py --output artifacts/reproduce-parameters --cases target-height-1.10,target-height-1.25,payload-mass-scale-1.2,rgbd-delay-0.05,rgbd-stale-0.15,calibration-bias-0.005,low-friction-0.7,tray-yaw-0.035 --seeds 0 --variants coop-depth-cad
.venv/bin/python tools/benchmark_mobile.py --output artifacts/reproduce-repeat --cases nominal --seeds 1 --variants coop-depth-cad,no-coop-depth-cad
.venv/bin/python tools/benchmark_mobile.py --output artifacts/reproduce-blue --cases nominal,gray-tray --seeds 0 --variants coop-blue-rgbd
```

전달 지연은 촬영 프레임·K·TF·시각을 고정한 상태에서 물리 시간을 진행시킨 합성 모델이다. 보정 편향은 추정 마운트 TF에만 적용한다. 제조사 카메라의 stereo 알고리즘이나 실물 노이즈·지연 분포를 재현했다고 주장하지 않는다.
