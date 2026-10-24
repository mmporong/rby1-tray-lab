# 전신 동기 응력 추정과 공식 CAD 플랜지 해석

1,136개 재생 프레임에서 전신의 등가 단면 응력 추정과 공식 STEP 플랜지의 준정적 FEM을 같은 영상 시각에 표시한다. 전신 표시는 제조사 전체 CAD의 요소 해석이 아니다. 플랜지는 6개 단위 wrench의 응력 텐서 기저를 현재 힘·모멘트로 합성해 표면과 실제 단면에 표시한다. 기본 화면은 구조 응력 위에 몸통·양팔 20관절의 구동기 부하를 겹치며, 기존 관절 부하 영상은 같은 시각의 비교 자료로 남긴다.

## 전신 동기 구조 모드

- 시각 범위는 37개 body와 63개 geom이다. 36개 body는 가정한 등가 원형 단면으로 축응력·굽힘·비틀림·전단을 계산한다. 실제 시각 외피는 위치와 자세를 보여 주며, 그 외피 자체를 구조 메시로 해석한 결과는 아니다.
- 질량이 없는 `NECK_0`는 내력이 0인 안전 부품으로 해석하지 않고 회색의 **비구조 외장·응력 미계산**으로 표시한다.
- 베이스는 자유 관절의 `cfrc_int`를 사용하지 않는다. torso와 네 바퀴 장착부 반력을 베이스 좌표계로 옮기고, 각 프레임에서 가정 단면의 순간 표면 peak가 큰 한 사례를 proxy로 표시한다. 실제 차체 FEA가 아니다.
- 왼쪽 플랜지는 별도 CAD FEM 상세다. 단위 Fx·Fy·Fz·Mx·My·Mz에 대한 응력 텐서 6성분 기저를 선형 합성한 뒤 von Mises를 계산한다. 비선형 스칼라인 von Mises 값을 하중별로 합산하지 않는다.
- 주 영상의 `requestVideoFrameCallback` `mediaTime`이 프레임 선택 기준이다. 미지원 브라우저에서는 영상 시간 이벤트로 보완한다. 시간·선택 부품·FEM 표면/단면 모드는 뷰 전환 뒤에도 유지한다.

원본 상태 1,136개와 물리 소스 커밋 `c7967a9aab6a31fafc574150d959c36477f9cf59`를 유지하고, 파생 동역학은 scratch 데이터에서만 계산했다. 정의된 관절 위치·토크, 베이스, 트레이와 적재물 위치·속도 및 단계 대조의 최대 차이는 0이다. 원본에 없는 적재물 회전은 대조하지 않았다. 손목 FT 교차검증 절댓값 최대는 `1.7763568394002505e-15`다. 기준 번들 생성은 136.403초가 걸렸다. [전신 요약](../evidence/structural-whole/summary.json), [수치 검증](../evidence/structural-whole/validation.json), [하중 진단 검증](../evidence/structural-whole/load-diagnostics.json), [브라우저 동기화 검증](../evidence/structural-whole/browser-qa.json)에 공개 가능한 근거를 둔다.

`cfrc_int`의 순서는 모멘트·힘이며, 세계축을 사용하고 원점은 해당 운동학 트리의 `subtree_com[body_rootid[b]]`다. 단면 원점 p로 옮길 때 `M_p = M_c + (c-p)×F`를 적용하고, 힘과 모멘트를 단면 좌표계로 회전한다. 렌더 시점에 맞춘 `mj_forward`와 `mj_rnePostConstraint`는 복제한 데이터에만 적용한다. 정의와 좌표 변환은 [MuJoCo 3.11.0 데이터 선언](https://github.com/google-deepmind/mujoco/blob/3.11.0/include/mujoco/mjdata.h), [공간 벡터 변환 소스](https://github.com/google-deepmind/mujoco/blob/3.11.0/src/engine/engine_util_spatial.c), [시뮬레이션 파이프라인](https://mujoco.readthedocs.io/en/3.11.0/programming/simulation.html)을 따른다.

등가 단면은 외피의 반경 범위를 감싸는 원형 관으로 가정하며, 두께는 `clamp(0.10R, 1 mm, 6 mm)`다. 실제 CAD의 내부 두께가 아니다. 축·굽힘 응력, 비틀림 전단과 공칭 평균 횡전단 `V/A`를 조합한다. 길이 방향으로 `Mx(z)=Mx(0)+z·Fy`, `My(z)=My(0)-z·Fx`를 적용한다. 분포 체적력·국부 체결 응력은 복원하지 않는다. 플랜지의 화면 수치는 표면/단면에 나타난 부분 최대가 아닌 전체 Tet4의 현재 최대값이다.

## 하중 경로와 응력 성분 진단

선택 부품의 단면 좌표에서 축력은 `N=Fz`, 횡전단력은 `V=hypot(Fx,Fy)`, 굽힘 모멘트는 `Mb=hypot(Mx,My)`, 비틀림은 `T=Mz`로 표시한다. 부호가 있는 `N`과 `T`, 각 축의 Fx·Fy·Fz·Mx·My·Mz도 함께 남긴다. 전신 등가 단면은 합성·축력·굽힘·횡전단·비틀림 응력을 바꿔 볼 수 있다. CAD FEM은 합성 von Mises, 법선 편차와 좌표축 전단으로 나눈다. FEM의 법선 편차/전단 분리는 선택한 좌표축에 따라 달라지고 두 값의 제곱합의 제곱근이 합성 von Mises가 된다. 여기서 좌표축 전단은 단면 횡전단력 `V`와 같은 항이 아니다.

모델 위의 힘·모멘트 화살표와 좌표축 문자는 표시하지 않는다. 힘·모멘트·기준 원점은 하중 진단 수치에서 확인하며, 최대점 지시선은 표시 외피에서 찾은 최대 응력 표본을 가리킨다. 전체 보기에서는 현재 프레임의 합성 응력이 큰 부품 세 개를 제시하며 버튼으로 해당 부품을 선택할 수 있다.

관절 오버레이는 몸통 6개와 양팔 14개 구동 관절의 기록을 사용한다. 관절 원은 그리지 않으며 전신에서 부하가 큰 3개 관절을 지시선과 수치 라벨로 가리킨다. 캔버스 폭이 500 px보다 작으면 1개, 부품 선택 시에는 해당 관절만 표시한다. CAD 상세에서는 지시선을 생략한다. 사용률은 `|현재 토크| / 모델 제한`이며 100%를 넘는 값도 그대로 표시한다. 20개 전체 관절의 부호 있는 토크(Nm), 모델 제한(Nm), 사용률(%)은 접을 수 있는 표에 유지한다. 지시선을 꺼도 수치 표는 유지한다. 모델 제한은 시뮬레이션의 활성·대칭 액추에이터 제한이며 실측 하드웨어 정격이 아니다. 응력 MPa 색상 범례와 관절 토크 수치는 별도 물리량이다.

현재 합력 `W`를 같은 원점과 좌표계에서 다음 세 항으로 회계한다.

- 자중 지지 `G=-Σm_i g`: 선택 절단면 아래 로봇 subtree의 질량에 대한 지지 하중이다.
- 외력 반작용 `E=-Σcfrc_ext`: subtree에 작용한 접촉·적용 외력·공간 구속 wrench를 절단 원점으로 옮긴 반작용이다.
- 운동 잔차 `I=W-G-E`: 현재 합력에서 자중 지지와 외력 반작용을 뺀 회계 잔차다.

외력과 내력의 누적 정의는 [MuJoCo 3.11.0 RNE 구현](https://github.com/google-deepmind/mujoco/blob/3.11.0/src/engine/engine_core_smooth.c#L2211-L2405)을 따른다. 세 항은 원인별 응력 기여율이 아니다. 부호 있는 응력의 상쇄와 6축 결합이 있으므로 항별 등가응력을 더해 합성 응력을 복원하지 않는다. 적재물 질량은 로봇 subtree 질량에 포함하지 않으며 접촉 외력으로 전달된다. 화면은 부모와 하부 구조 사이의 경로와 포함된 로봇 질량을 함께 표시한다. 베이스 proxy는 프레임마다 선택되는 장착부가 달라질 수 있어 경로와 포함 질량도 프레임별로 기록한다.

일반 절단면의 `W`는 parent가 child subtree를 지지하는 `parent_on_child` 부호다. 베이스 proxy와 CAD 플랜지는 하부가 상부에 가하는 반작용인 `child_on_parent` 부호를 쓴다. 1,136×38×18개의 float32 값에는 G·E·I의 6축 wrench가 들어간다. 현재 구조 번들의 `diagnostics.bin`은 3,108,096 B다. 정의된 원본 필드의 최대 차이는 0, 현재 wrench 대조의 최대 절댓값 차이는 `3.049538e-05`, `W-(G+E+I)` 재구성 잔차는 `6.103515625e-05`다. 이 수치는 float32 저장 정밀도를 포함한다.

## 플랜지 대상과 확인 범위

- 원본: RB-Y1 V1.2 Mecanum의 `Fast Grip` STEP. SHA-256은 `387276a46fccbce4a48f50928710633305f525ec3aff825deabb086c8d524854`다.
- Gmsh 4.15.2/OpenCASCADE로 m 단위 가져오기 후 volume tag 1672를 선택했다. 왼쪽 손목과 그리퍼 사이의 원형 연결 플랜지 후보이며, 제조사 부품 번호·재료 카드는 제공되지 않았다.
- 선택 솔리드의 외경은 63 mm, 두께는 11 mm, OCC 체적은 30,458.833 mm³다. 면 21개와 홈·블라인드 홀을 포함한다. 추출 BREP을 다시 열어 1개 volume·21개 면을 확인했다.
- 전체 가져오기의 volume 엔터티 1,676개는 실제 부품 수가 아니다. 이름이 없는 엔터티도 있어 tag·위치·치수·체적·원본 해시를 함께 고정했다. 선택 tag만으로 다른 STEP에 적용하지 않는다.
- 양의 체적, 3D 메시 생성과 양의 요소 Jacobian을 확인했다. 전체 제조사 CAD의 무결성 인증을 뜻하지 않는다.

공식 CAD의 Fast Grip 손가락과 현재 Menagerie 손가락의 치수가 다르다. 이 해석은 **다른 구성의 CAD 부품에 시뮬레이션 하중을 전달하는 연구**다. 실물과 같은 조립체의 검증 결과로 해석하지 않는다.

## 정적 검증 사례의 입력 하중과 경계조건

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

## 정적 검증 사례의 수치 결과와 검증

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

사이트는 화이트 톤의 패널·표·조작부와 짙은 텍스트를 사용한다. 응력·깊이 색상과 디지털 트윈 장면은 데이터의 색상 체계를 유지한다. 첫 화면은 원본 동작과 전신 구조 응력·관절 부하 통합 화면의 헤더와 영상 높이를 맞춘다. 공통 재생 제어를 두 영상 아래에 두고, 표시 설정과 관절 부하 요약을 그 다음 행에 나란히 배치한다. 통합 화면의 회청색 디지털 트윈 바닥은 0.25 m 간격 격자이며 로봇과 응력 색상을 읽기 위한 시각 배경이다. 물리 충돌 형상은 추가하지 않는다. 선택 부품의 하중 진단은 그 아래 전체 폭에 놓고, 세부 전달 하중 표는 기본으로 접는다. RGB·깊이는 넓은 화면에서 카메라별 3열로 놓고 좁은 화면에서는 세로로 배치한다. 기본 화면은 전신 등가 단면 응력과 20관절 부하를 함께 표시하며, 체크박스로 관절 지시선만 감출 수 있다. 목록이나 현재 고응력 부품 세 개 중 하나를 고르면 부품을 격리해 확대한다. 왼쪽 플랜지 CAD 상세에서는 위치 지시선을 생략하고 관절 수치 표는 유지한다. 기존 관절 부하 영상은 비교 모드로 전환해 볼 수 있다. 단면은 `x+y=0` 평면과 실제 Tet4의 교차다. 홈·홀을 임의로 메우거나 요소 안에 임의 응력 경사를 만들지 않는다. 실제 형상 크기로 표시하며 변형은 확대해 그리지 않는다.

전신과 플랜지 하중은 영상의 같은 기록 프레임을 사용한다. 결과는 기록 하중을 이용한 단방향 후처리이며, 원본 시뮬레이션의 위치·질량·관성·접촉·제어에 되돌려 적용하지 않는다. 71.704초의 정적 플랜지 결과는 메시 수렴과 경계조건 민감도 검증 사례로 보존한다.

STEP/BREP/NPZ와 CAD 파생 데이터가 포함된 동기 구조 번들의 재배포 허용 범위가 확인되지 않아 `artifacts/` 아래 로컬에만 둔다. README의 플랜지 응력 그림은 이 번들로 뷰어를 실행해 캡처한 이미지 한 장이며 번들·STEP·BREP·NPZ는 포함하지 않는다. 제작 기록은 [미디어 출처](media/provenance.json)에 있다. **새 clone에서는 동기 구조 모드가 비활성**이며, 원본 3개 영상·관절 부하·RGB-D 재생은 계속 사용할 수 있다. 구조 번들이 아예 없을 때만 기존 71.704초 고정 스냅샷을 명시해 표시한다.

서버는 필수 바이너리 네 개와 선택 진단 바이너리 `diagnostics.bin`만 고정 경로로 제공한다. 선택 파일도 manifest에 기록한 해시·스키마·크기를 검사한다. 하중 진단을 덧붙인 번들은 기존 네 바이너리의 바이트를 바꾸지 않는다. 구성된 번들이 검증에 실패하면 구조 패널에 오류를 표시하고 고정 스냅샷으로 우회하지 않는다. CAD·NPZ와 내부 파일 경로는 다운로드 경로로 노출하지 않는다.

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
.venv-cae/bin/python tools/build_structural_basis.py \
  --input artifacts/structural-flange/level-2.npz \
  --output artifacts/structural-basis
.venv/bin/python tools/build_whole_structural.py --output artifacts/structural-whole
.venv/bin/python tools/build_load_diagnostics.py \
  --scene artifacts/structural-whole \
  --output artifacts/structural-with-load-path
MUJOCO_GL=egl .venv/bin/python -m unittest discover -s tests -v
.venv/bin/python -m rby1_tray.cli view \
  --structural-scene-directory artifacts/structural-with-load-path
```

각 `--output` 경로는 실행 전에 존재하지 않아야 한다. 하중 진단 출력은 원본 구조 번들을 읽어 새 디렉터리에 다섯 번째 파일과 갱신한 manifest를 만들며, 기존 네 바이너리는 그대로 복사한다. 수동으로 기본 폴더를 교체할 필요 없이 `--structural-scene-directory`에 새 출력 경로를 지정한다. `view`는 옵션이 없으면 기본 경로 `artifacts/structural-whole`을 확인한다. 원본 STEP은 ZIP 안의 하위 폴더에 있으므로 압축 해제 위치가 다르면 `--step` 경로를 바꾼다. 선택 형상은 원본 해시·bbox·체적으로 검사한다. Gmsh 가져오기·visible-only BREP 내보내기와 메시 API는 [공식 Gmsh 4.15.2 매뉴얼](https://gmsh.info/doc/texinfo/gmsh.html)을 따른다. Gmsh의 GPL-2.0-or-later 및 upstream 링크 예외는 [공식 배포 조건](https://gmsh.info/)을 확인한다. Gmsh를 프로젝트 코드의 MIT 라이선스로 재배포하지 않는다.
