# RB-Y1 구조 CAD 배포처와 확인 범위

공식 **3D CAD 배포 목록은 존재한다.** 현재 프로젝트의 Menagerie 표면 메시와 제조사 3D CAD ZIP을 구분한다. 부품 내부 응력의 계산·표시는 아직 구현 완료 결과가 아니다.

## 공식 CAD

[Rainbow Robotics RB-Y1 제품 페이지](https://www.rainbow-robotics.com/en/products/rb-y1)의 다운로드 목록에는 다음 3D ZIP 5종이 있다. [공식 SDK Discussion #54](https://github.com/RainbowRobotics/rby1-sdk/discussions/54)에서도 제조사 contributor가 2025년 2월 19일 공식 다운로드 페이지의 3D CAD를 안내했다.

| 버전 | 베이스 | 공식 표시 파일명 |
|---|---|---|
| V1.0 | Mecanum | `RB-Y1_V1.0_3D_Mecanum-wheel-typed-Mobile.zip` |
| V1.1 | Differential | `RB-Y1_V1.1_3D_Differential-gear-typed-Mobile.zip` |
| V1.1 | Mecanum | `RB-Y1_V1.1_3D_Mecanum-wheel-typed-Mobile.zip` |
| V1.2 | Differential | `RB-Y1_V1.2_3D_Differential-gear-typed-Mobile.zip` |
| V1.2 | Mecanum | `RB-Y1_V1.2_3D_Mecanum-wheel-typed-Mobile.zip` |

현재 시뮬레이션은 RBY1-M 1.2 계열이므로 V1.2 Mecanum 자료가 우선 확인 대상이다. 이는 버전·베이스 대응이며 ZIP의 구조해석 적합성까지 확인했다는 뜻은 아니다.

다운로드 게이트는 개인이면 이름·이메일·휴대전화, 사업자이면 회사명·담당자 이름·이메일·휴대전화와 개인정보 수집·이용 동의를 요구한다. 동의 없는 파일 요청은 HTTP 403이었다. 개인정보 제출이나 동의는 수행하지 않았으며 ZIP을 확보하지 못했다. 따라서 **파일 크기·내부 포맷·STEP/솔리드 포함 여부·내부 형상·CAD 라이선스는 미확인**이다. SDK의 Apache-2.0을 이 ZIP에 적용하지 않는다. 공식 원본 파일 경로와 확인 결과는 [조사 JSON](../evidence/cad-availability.json)에 보존했다.

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

이는 MuJoCo 하중을 별도 구조해석에 전달하는 단방향 후처리다. 계산한 변형을 강체 동역학에 되돌리는 연성 해석은 현재 범위에 포함하지 않는다. 구조 CAD를 확보한 뒤에도 내부 구조·물성·체결 정보가 충분한지는 별도 판단해야 한다.
