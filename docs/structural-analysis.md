# 공식 CAD 플랜지의 정적 하중 전달 연구

MuJoCo의 손목 6축 기록을 공식 STEP의 개별 솔리드에 전달하고, 선형탄성 FEM으로 부품 내부 응력과 변위를 계산한다. 표면은 경계 삼각형에 인접한 요소 응력을, 단면은 사면체와 평면의 교차 영역을 표시한다. 한 부품의 모든 면에 같은 색을 칠하는 관절 부하 모드와 별도로 제공한다.

## 대상과 확인 범위

- 원본: RB-Y1 V1.2 Mecanum의 `Fast Grip` STEP. SHA-256은 `387276a46fccbce4a48f50928710633305f525ec3aff825deabb086c8d524854`다.
- Gmsh 4.15.2/OpenCASCADE로 m 단위 가져오기 후 volume tag 1672를 선택했다. 왼쪽 손목과 그리퍼 사이의 원형 연결 플랜지 후보이며, 제조사 부품 번호·재료 카드는 제공되지 않았다.
- 선택 솔리드의 외경은 63 mm, 두께는 11 mm, OCC 체적은 30,458.833 mm³다. 면 21개와 홈·블라인드 홀을 포함한다. 추출 BREP을 다시 열어 1개 volume·21개 면을 확인했다.
- 전체 가져오기의 volume 엔터티 1,676개는 실제 부품 수가 아니다. 이름이 없는 엔터티도 있어 tag·위치·치수·체적·원본 해시를 함께 고정했다. 선택 tag만으로 다른 STEP에 적용하지 않는다.
- 양의 체적, 3D 메시 생성과 양의 요소 Jacobian을 확인했다. 전체 제조사 CAD의 무결성 인증을 뜻하지 않는다.

공식 CAD의 Fast Grip 손가락과 현재 Menagerie 손가락의 치수가 다르다. 이 해석은 **다른 구성의 CAD 부품에 시뮬레이션 하중을 전달하는 연구**다. 실물과 같은 조립체의 검증 결과로 해석하지 않는다.

## 입력 하중과 경계조건

기존 `evidence/mobile-native/controller-data/commands.jsonl.gz`의 500 Hz 패킷을 사용한다. `retract_low`, `drive_loaded`, `raise_high`, `drive_high` 중 왼쪽 손목의 raw torque norm이 가장 큰 패킷을 FEM 실행 전에 고른다. 선택 시점은 `raise_high`의 **71.704 s**, 패킷 index 35,852다. FEM 응력이 가장 큰 시점을 탐색한 결과는 아니다.

| 항목 | 설정 |
|---|---|
| Raw FT 힘, 센서 좌표 | −1.172331, +1.318480, +7.742224 N |
| Raw FT 모멘트, 센서 원점 | +0.113362, +0.347938, −0.336696 N·m |
| CAD 하중 힘 | +1.172331, −1.318480, −7.742224 N |
| CAD 하중 모멘트, 하단 면 중심 | −0.113362, −0.347938, +0.336696 N·m |
| 재료 가정 | 등방 선형탄성, E=69 GPa, ν=0.33 |
| 기본 지지 | 상단 평면의 모든 노드, 병진 3축 0 |
| 하중 패치 | 하단 평면 면적/3으로 노드 가중치 생성, 6축 결과력·모멘트를 보존하는 분포 |
| 좌표 전달 가정 | 센서→CAD 회전 I, 센서 원점=CAD 하단 면 중심, offset=0 |
| 체결 민감도 | 상단 외곽 r≥25 mm만 고정하여 같은 메시·하중으로 재계산 |

[MuJoCo 3.11.0 force/torque 문서](https://mujoco.readthedocs.io/en/3.11.0/XMLreference.html#sensor-force)와 [3.11.0 센서 소스](https://github.com/google-deepmind/mujoco/blob/b85fdca54f0e0038b804af146a0b4e94199e00d0/src/engine/engine_sensor.c#L1279)에 따라 site 좌표·site 원점의 wrench를 사용한다. parent가 child subtree를 지지하는 raw wrench를 반전하여 하부 subtree가 proximal CAD 부품에 가하는 반작용으로 적용한다.

일반 전달식은 `F = −R F_sensor`, `M = −[R M_sensor + (O_site−O_load) × (R F_sensor)]`다. 실제 설정의 동축·원점 일치는 측정값이 아닌 가정이다. 회전·원점이 다른 시험에서는 명시적 nodal force들의 결과력·모멘트와 대조한다.

볼트 프리로드, 마찰·분리 접촉, 해당 CAD 부품의 자중·관성 체적력, 동적 진동, 소성·피로는 포함하지 않는다. 기록 wrench가 전달하는 하부 subtree의 동역학적 결과력과 CAD 부품 자체의 관성 응답을 구분한다. 제조사 정격·허용 응력·실물 안전율은 산출하지 않는다.

## 수치 결과와 검증

| 목표 메시 길이 | 노드 | Tet4 | 최대 변위 | 체적 가중 P95 von Mises | 요소 최대 von Mises |
|---|---:|---:|---:|---:|---:|
| 2.8 mm | 2,577 | 9,736 | 4.794 nm | 0.020112 MPa | 0.033856 MPa |
| 2.0 mm | 5,116 | 21,048 | 4.761 nm | 0.020569 MPa | 0.035355 MPa |
| 1.4 mm | 12,165 | 55,946 | 4.844 nm | 0.020905 MPa | 0.038520 MPa |

마지막 두 메시 사이의 상대 변화는 최대 변위 1.694%, 체적 가중 P95 1.608%, 변형 에너지 1.471%로 설정한 5% 기준을 통과했다. 이는 세 관심 수치의 메시 민감도 기준이다. 국부 요소 최대 응력은 약 8.2% 증가했으며 수렴 판정에 포함하지 않는다. 완전 고정면과 모서리 부근 최대값을 파손 기준으로 사용하지 않는다.

미세 메시의 체적은 30,481.412 mm³로 OCC 체적과 약 0.074% 차이다. 최소 Jacobian은 양수이며 scaled inverse condition number의 최소값은 0.260이다. 상대 힘 평형 오차는 `6.52e-15`, 모멘트는 `8.32e-16`, `2U=Fᵀu` 에너지 항등 상대 오차는 `7.47e-16`이다. 수치적 평형이 실제 재료·체결 가정의 정확성을 보증하지 않는다.

상단 외곽만 고정하면 최대 변위는 **6.702 nm**, P95는 **0.025643 MPa**, 요소 최대는 **0.106900 MPa**가 된다. 고정 조건에 따라 국부 최대 응력이 약 2.78배 달라진다. 이 민감도는 한 메시에서 수행한 경계조건 비교이며 별도의 수렴 판정 결과는 아니다.

검증은 affine constant-strain 패치, 축방향 해석식 변위 대조, 하중 선형성, 좌표 이동 공변성, 6축 하중 분포의 원점 공변성, 강체 모드 구속, 평형과 에너지 항등을 포함한다. 원본 NPZ 해시와 요소 수·최대/P95 응력·변위를 리포트와 대조한 뒤 렌더하며, 필드 변조와 수치 불일치를 거부한다.

수치 요약과 로컬 산출물 해시는 [공개 요약](../evidence/structural-flange/summary.json)에 보존한다.

## 화면과 배포 범위

상단 원본 영상 아래 RGB·깊이 모니터 옆에서 **부품 응력 → 표면/단면**을 선택한다. 단면은 `x+y=0` 평면과 실제 Tet4의 교차다. 홈·홀을 임의로 메우거나 요소 안에 임의 응력 경사를 만들지 않는다. 실제 형상 크기로 표시하며 변형은 확대해 그리지 않는다.

응력은 71.704초의 고정 시점 결과다. 영상 재생 시각에 따라 변하는 동적 응력처럼 표시하지 않는다. 원본 시뮬레이션의 위치·질량·관성·접촉·제어에는 이 결과를 되돌려 적용하지 않는다.

STEP/BREP/NPZ와 CAD 렌더 PNG의 재배포 허용 범위가 확인되지 않아 이 자료는 `artifacts/` 아래 로컬에만 둔다. 저장소에는 계산 코드·설정·수치 요약·해시·재현 절차를 포함한다. **새 clone에서는 구조 모드가 비활성**이며, CAD를 확보해 결과를 생성하면 나타난다. 원본 3개 영상·관절 부하·RGB-D 재생은 계속 사용할 수 있다.

뷰어는 고정된 PNG 2개만 제공한다. 보고서·파일 해시·형식·픽셀 크기·심볼릭 링크를 검사하며, 오류가 있으면 구조 모드를 격리한다. 서버는 CAD·NPZ를 다운로드 경로로 노출하지 않는다.

## 재현

먼저 저장소 README의 기본 환경과 제조사 동의 절차를 거쳐 V1.2 Mecanum STEP을 준비한다. 아래 출력 폴더는 없어야 한다. `requirements-cae.txt`는 기존 MuJoCo 환경과 분리된 내보내기 환경용이다.

```bash
cd "$HOME/rby1-tray-lab"
uv venv .venv-cae --python .venv/bin/python
uv pip install --python .venv-cae/bin/python numpy==2.5.2 scipy==1.18.1 -r requirements-cae.txt
.venv-cae/bin/python tools/prepare_structural_cad.py \
  --step "$HOME/Downloads/RB-Y1_v1.2_3D_Fast Grip_MW_KCs_260107.step" \
  --output artifacts/structural-cad-selected
.venv-cae/bin/python tools/build_structural_case.py \
  --brep artifacts/structural-cad-selected/left-interface-flange.brep \
  --output artifacts/structural-flange
.venv-cae/bin/python tools/render_structural_case.py --directory artifacts/structural-flange
MUJOCO_GL=egl .venv/bin/python -m unittest discover -s tests -v
.venv/bin/python -m rby1_tray.cli view --structural-directory artifacts/structural-flange
```

원본 STEP은 ZIP 안의 하위 폴더에 있으므로 압축 해제 위치가 다르면 `--step` 경로를 바꾼다. 선택 형상은 원본 해시·bbox·체적으로 검사한다. Gmsh 가져오기·visible-only BREP 내보내기와 메시 API는 [공식 Gmsh 4.15.2 매뉴얼](https://gmsh.info/doc/texinfo/gmsh.html)을 따른다. Gmsh의 GPL-2.0-or-later 및 upstream 링크 예외는 [공식 배포 조건](https://gmsh.info/)을 확인한다. Gmsh를 프로젝트 코드의 MIT 라이선스로 재배포하지 않는다.
