# 실행 증거

코드 커밋 `8342a8079f6181eaa18a4a774d786ba0e4ccb772`에서 생성한 CPU MuJoCo 실행입니다. 모든 실행의 소스 해시와 의존성 버전은 각 `manifest.json`에 있습니다. 이 디렉터리를 추가한 커밋에서도 실행 소스는 같은 해시를 유지합니다.

## 구성

| 경로 | 조건과 결과 |
|---|---|
| `demo/` | 기본 높이 0.75 m, 시드 0, 전체 동작 성공 및 약 4배속 GIF |
| `dataset/` | 지지면 0.75/0.80/0.85 m × 시드 0/1/2, 사전 등록 9개 조합 |
| `ablations/fast/` | 이동 시간 배율 1, 안착 전 파지 이탈 |
| `ablations/fixed/` | 몸통 자세 고정, 상승 경로 충돌로 계획 단계 중단 |
| `ablations/heavy/` | 적재물 합계 0.270 kg, 시드 0, 전체 동작 성공 |
| `experiments.json` | 비교 실행의 결과와 매니페스트 참조 |

0.75 m는 3/3 성공, 0.80 m는 3/3 안착 전 파지 이탈, 0.85 m는 3/3 IK 경로 조건 불충족입니다. 0.80 m의 최초 실패 시각은 각 실행에서 62.596초입니다. 시드가 바꾸는 값은 적재물 초기 XY 위치의 ±4 mm 변화입니다.

기본 실행의 완료 시각은 84.024초, 최대 관절 추종 오차는 0.002845 rad, 최대 관절 구동 토크는 248.084 N·m입니다. 트레이 중심이 도달한 최고 높이 0.864585 m는 운반 경유점의 값이며 컨테이너 지지면 높이 0.75 m와 구분합니다.

## 원본과 검사

`trace.jsonl.gz`는 원본 JSONL 바이트를 gzip으로 압축한 파일입니다. 각 실행의 `manifest.json`은 압축 전 원본 trace를 비롯한 결과물의 SHA-256을 저장합니다. 아래 도구가 압축 스트림을 읽어 원본 해시를 검사합니다. 로봇 메시 자체는 이 저장소에 포함하지 않습니다.

```bash
python3 tools/verify_evidence.py evidence/dataset evidence/demo \
  evidence/ablations/fast evidence/ablations/fixed evidence/ablations/heavy
```

검사는 실행별 파일·소스 해시, 마지막 trace와 결과의 일치, 사전 등록한 높이×시드 조합의 완전성, 집계와 실행별 결과의 일치를 확인합니다. `dataset/dataset-manifest.json`은 사전 등록·집계·각 실행 매니페스트를 묶습니다. 해시는 보관한 바이트의 일관성을 확인하는 용도이며 외부 기관의 인증을 뜻하지 않습니다.

## 재실행

저장소 루트에서 설치·에셋 준비를 마친 뒤 실행합니다. 새 출력 경로를 사용해야 합니다.

```bash
.venv/bin/python -m rby1_tray.cli benchmark \
  --heights 0.75,0.8,0.85 --seeds 0,1,2 --output artifacts/reproduce-dataset
.venv/bin/python -m rby1_tray.cli run \
  --config evidence/ablations/fast/config.json --output artifacts/reproduce-fast
.venv/bin/python -m rby1_tray.cli run \
  --config evidence/ablations/fixed/config.json --output artifacts/reproduce-fixed
.venv/bin/python -m rby1_tray.cli run \
  --config evidence/ablations/heavy/config.json --output artifacts/reproduce-heavy
```

실패를 포함한 벤치마크와 실패 비교군의 종료 코드 1은 기록된 판정 결과입니다. 모델·제어·적재물 조건을 바꾸면 새 실행으로 구분합니다. 물리 판정은 2 ms마다 수행하고 저장 로그는 20 ms 간격과 종료 이벤트 시점에 남깁니다. 빠른 궤적·높이 변화에서 검출한 접촉 이탈도 같은 판정기를 사용했습니다.
