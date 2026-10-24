# RB-Y1 구조 CAD 배포처와 확인 범위

공식 **V1.2 Mecanum 3D CAD ZIP과 STEP 파일을 확보했다.** 현재 프로젝트의 Menagerie 표면 메시와 제조사 STEP을 구분한다. 전신 36개 부품은 Menagerie 시각 외피에 대응한 가정 단면 추정이며 제조사 전체 CAD FEM이 아니다. 제조사 STEP은 선택한 왼쪽 연결 플랜지 후보 한 부품의 준정적 요소 응력·표면·단면 계산에만 사용한다. 범위와 동기화 방식은 [전신 동기 응력 추정과 공식 CAD 플랜지 해석](structural-analysis.md)에 기록한다.

## 공식 CAD

[Rainbow Robotics RB-Y1 제품 페이지](https://www.rainbow-robotics.com/en/products/rb-y1)의 다운로드 목록에는 다음 3D ZIP 5종이 있다. [공식 SDK Discussion #54](https://github.com/RainbowRobotics/rby1-sdk/discussions/54)에서도 제조사 contributor가 2025년 2월 19일 공식 다운로드 페이지의 3D CAD를 안내했다.

| 버전 | 베이스 | 공식 표시 파일명 |
|---|---|---|
| V1.0 | Mecanum | `RB-Y1_V1.0_3D_Mecanum-wheel-typed-Mobile.zip` |
| V1.1 | Differential | `RB-Y1_V1.1_3D_Differential-gear-typed-Mobile.zip` |
| V1.1 | Mecanum | `RB-Y1_V1.1_3D_Mecanum-wheel-typed-Mobile.zip` |
| V1.2 | Differential | `RB-Y1_V1.2_3D_Differential-gear-typed-Mobile.zip` |
| V1.2 | Mecanum | `RB-Y1_V1.2_3D_Mecanum-wheel-typed-Mobile.zip` |

현재 시뮬레이션은 RBY1-M 1.2 계열이므로 V1.2 Mecanum 자료를 확보했다. ZIP의 실제 STEP 파일명에는 `Fast Grip`, `MW`, `KCs`가 포함된다. 동봉 명명 가이드상 고속 그리퍼·메카넘 베이스·KC 인증 구성 표기다. 그리퍼 형상과 프레임을 현재 시뮬레이터 구성과 대조해야 하며, 파일명만으로 동일 부품이나 구조해석 적합성을 확정하지 않는다.

다운로드 게이트는 개인이면 이름·이메일·휴대전화, 사업자이면 회사명·담당자 이름·이메일·휴대전화와 개인정보 수집·이용 동의를 요구한다. 최초 익명 요청은 HTTP 403이었다. 이후 사용자가 브라우저에서 동의 양식을 완료했지만, 제품·동의 페이지는 `www.rainbow-robotics.com`, 파일 링크는 `rainbow-robotics.com`으로 달라 기본 링크는 계속 권한 없음 화면을 표시했다. 같은 경로·파일 식별자를 동의한 `www` 호스트로 요청하자 다운로드됐다. 쿠키를 복제하거나 권한을 바꾸지 않았고 개인정보도 재제출하지 않았다.

공식 원본 경로와 성공한 주소, 파일 해시는 [조사 JSON](../evidence/cad-availability.json)에 보존했다. 원본 ZIP·STEP·동봉 PNG는 로컬에만 보관한다. 공개 저장소에는 뷰어에서 플랜지 응력 결과를 캡처한 그림 한 장(`docs/media/flange-fem-stages.png`)만 싣고, 제작 기록은 [미디어 출처](media/provenance.json)에 둔다. CAD 재배포 라이선스는 여전히 미확인이며 SDK의 Apache-2.0을 적용하지 않는다.

## 확보한 STEP의 초기 검사

| 항목 | 확인 결과 |
|---|---|
| ZIP | `RB-Y1_V1.2_3D_Mecanum-wheel-typed-Mobile.zip`, 19,607,169 bytes, CRC 전체 검사 PASS |
| STEP | `RB-Y1_v1.2_3D_Fast Grip_MW_KCs_260107.step`, 98,877,141 bytes |
| 동봉 파일 | 명명 규칙 PNG 1개, 42,618 bytes |
| STEP 스키마 | AP214 `AUTOMOTIVE_DESIGN` |
| 솔리드 관련 텍스트 엔티티 | `MANIFOLD_SOLID_BREP` 1,577개, `BREP_WITH_VOIDS` 93개 |
| 셸 관련 텍스트 엔티티 | `CLOSED_SHELL` 2,460개, `OPEN_SHELL` 10개 |
| 재료 지정 텍스트 엔티티 | `MATERIAL_DESIGNATION` 0개 |

이는 STEP 문법의 엔티티 목록 검사다. **실제 부품 수·유효한 솔리드 수·재료 물성을 뜻하지 않는다.** 이후 Gmsh 4.15.2/OpenCASCADE로 m 단위 가져오기를 수행했으며 volume 엔터티는 1,676개였다. 왼쪽 손목 부근의 tag 1672를 위치·bbox·체적·원본 SHA로 고정하고 별도 BREP으로 추출했다. 다시 읽은 BREP에는 1개 volume·21개 면이 있고 외경 63 mm, 두께 11 mm, 체적 30,458.833 mm³다. 실제 제조사 부품 번호는 확인되지 않았다.

이 선택 부품의 양의 OCC 체적과 사면체 메시의 양의 Jacobian을 확인하고 선형탄성 응력을 계산했다. 전체 CAD의 무결성 인증이나 제조사 체결·재료 확인은 수행하지 않았다. 공식 Fast Grip과 현재 시뮬레이터 손가락의 치수가 달라, 동축·동일 원점을 가정한 하중 전달 연구로 한정한다. 계산 결과·메시 수렴·체결 민감도와 재현 절차는 [구조해석 문서](structural-analysis.md), 수치 원본은 [공개 요약](../evidence/structural-flange/summary.json)을 참고한다.

## 공개 모델을 실제 검사한 결과

[공식 SDK v0.10.0 릴리스](https://github.com/RainbowRobotics/rby1-sdk/releases/tag/v0.10.0)의 `rby1_sdk-0.10.0.tar.gz`를 내려받아 archive 멤버명을 전수 확인했다. 크기는 93,367,340 bytes이며 SHA-256은 다음과 같다.

```text
441af9e1d852382473aacdcb0526546e7f6e95bb50f84fdddf2c6b8328d50dd8
```

| 확인 항목 | 결과 |
|---|---|
| 표면 모델·시뮬레이터 파일 | OBJ 2,081개, DAE 107개, GLB 98개, URDF 11개, XML 342개 |
| 검사한 CAD·체적 FEM 확장자 | STEP/STP, IGES/IGS, Parasolid, BREP, MSH, INP, BDF 등 0개 |
| 공식 SDK·ROS 모델의 역할 | URDF/MJCF의 시각·충돌 형상, 질량·관성·관절 정의 |
| 부품별 구조 데이터 | 재료 카드·탄성계수·두께·체결 조건을 확인하지 못함 |

확장자 개수는 archive 멤버명 기준이다. 제조사 게이트 뒤 CAD ZIP 내용에 대한 판별이 아니다. 공식 `rby1-ros2`, `rby1_ros2_moveit`, `rby1-sim-isaac`와 Menagerie 미러도 조사했으며 확인한 커밋·포맷·범위를 JSON에 기록했다.

공식 SDK의 `LINK_6` 시각 OBJ 6개는 정점 위치 중복을 병합한 뒤에도 모두 `is_watertight=false`, `is_volume=false`였다. UV·법선 seam의 중복 정점 영향을 받는 raw edge 개수는 별도 진단값으로만 남겼다. 구멍을 채우거나 면을 추가하지 않았다. 이 6개 검사로 로봇의 모든 부품을 일반화하지 않으며, 시각 외피를 임의로 닫는 것만으로 실제 중공·리브·베어링 시트가 복원되지는 않는다.

공식 제품 페이지는 외장 재료를 Aluminum, Neck Joint Module의 출력단 재료를 AL6061로 표시한다. 이 정보를 모든 팔·몸통 링크의 합금·열처리·물성으로 확대하지 않는다.

## 부품 내부 응력 표시로 연결할 조건

1. 확보한 ZIP의 라이선스·archive 목록·버전·단위를 확인하고, 하중을 받는 부품의 솔리드와 내부 형상을 식별한다.
2. 부품별 재료, 두께, 베어링·플랜지·볼트 체결 및 접촉 경계를 정한다. 가정값은 제조사 실측값과 구분한다.
3. 선택한 MuJoCo 시점의 힘·모멘트를 구조 좌표계로 변환한다. 동적 하중을 주장하려면 관성 하중의 포함 범위도 정한다.
4. FEM 요소별 응력·변형을 계산하고, 반력 평형·에너지·메시 수렴·경계조건 민감도를 검사한다.
5. 같은 부품 안의 요소별 값을 표면 컬러맵과 단면에 표시한다. 현재 관절 토크/모델 제한 표시와 구분한다.

현재 구현은 선택 플랜지의 형상 추출, 가정을 명시한 재료·체결·좌표 전달, Tet4 계산과 평형·에너지·메시 민감도, 6개 단위 wrench 응력 텐서 기저, 기록 프레임별 표면·단면 표시까지 수행한다. 텐서 성분을 합성한 뒤 von Mises를 계산하며 하중별 스칼라 von Mises를 합산하지 않는다. MuJoCo 하중을 별도 구조해석에 전달하는 준정적 단방향 후처리이고, 변형을 강체 동역학에 되돌리는 연성 해석은 포함하지 않는다. 실제 재료·체결 및 Fast Grip과 시뮬레이터 사이의 프레임 정합은 여전히 확인되지 않았다.
