# 다른 PC에서 RB-Y1 사이트 실행하기

다른 PC에 저장소와 Python 환경을 설치하고 웹 서버를 실행하는 방법이다. **현재처럼 전신 응력·관절 부하·하중 진단까지 보려면 Git 저장소 외에 구조 번들 6개 파일이 필요하다.** 기존 실행 PC를 계속 사용할 수 있다면 [SSH 터널로 접속](#5-기존-pc의-화면을-다른-pc에서-보기)해도 된다.

## 1. 준비할 것과 실행 범위

| 항목 | 기본 재생 사이트 | 현재와 같은 전신 응력 화면 |
|---|---|---|
| Git 저장소 | 필요 | 필요 |
| Python 3.12 가상환경·MuJoCo 3.11.0 | 필요 | 필요 |
| Menagerie 로봇 모델 | `fetch-assets`로 별도 다운로드 | 동일 |
| 원본·관절 부하 영상·머리/양 손목 RGB·깊이 | Git에 포함 | 동일 |
| 전신 구조 응력·부품 선택·CAD 상세·하중 경로 | 구조 번들 없이는 비활성 | 구조 번들 별도 준비 |
| Node.js·npm·FFmpeg·Gmsh | 기존 자료 재생에는 불필요 | 완성된 구조 번들 재생에는 불필요 |

- 우선 실행 환경은 **Ubuntu 24.04 + Python 3.12 + Linux EGL**이다. 검증 범위는 [아래](#8-검증-범위)에 기록한다.
- Windows에서는 **WSL2의 Ubuntu 24.04** 안에 설치하고 Windows 브라우저에서 접속하는 경로를 안내한다. 이 문서 작성 시 WSL2 장비에서의 실행은 검증하지 않았다.
- Windows·macOS 네이티브 실행은 이 프로젝트에서 미검증이다. 현재 CLI에는 Linux EGL을 기본으로 선택하는 코드가 있어, Python 설치만으로 동일하게 실행된다고 보장하지 않는다. macOS 사용자는 검증된 Linux PC에 [SSH 터널로 접속](#5-기존-pc의-화면을-다른-pc에서-보기)할 수 있다.
- 설치 중 인터넷 연결이 필요하다. 브라우저는 H.264 영상과 WebGL을 사용할 수 있어야 한다.
- 다른 PC의 `.venv`를 복사하지 말고 새 가상환경을 만든다. **HTML 파일만 열거나 정적 파일 서버만 실행하면 API가 없어 작동하지 않는다.**

## 2. 새 PC에 설치

### Windows만: WSL2 준비

관리자 PowerShell에서 실행한다. 이미 Ubuntu 24.04가 있다면 설치를 건너뛴다.

```powershell
wsl --install -d Ubuntu-24.04
```

요청되면 재부팅한 뒤 Ubuntu를 열어 Linux 사용자와 암호를 만든다. PowerShell의 `wsl --list --verbose`에서 배포판의 `VERSION`이 `2`인지 확인한다. 이후 명령은 모두 **Ubuntu 터미널**에서 실행한다. 프로젝트는 Windows의 `/mnt/c` 대신 Linux 홈에 둔다. 설치 조건과 문제 해결은 [Microsoft WSL 설치 안내](https://learn.microsoft.com/windows/wsl/install)를 따른다.

### Ubuntu 공통: 시스템 패키지와 저장소

아래는 Ubuntu 24.04의 Mesa EGL 실행을 위한 설치 구성이다. NVIDIA 전용 렌더링 환경 등에서는 해당 드라이버 구성이 추가로 필요할 수 있다.

```bash
sudo apt update
sudo apt install -y git python3.12 python3.12-venv libegl1 libegl-mesa0 libgl1-mesa-dri

cd "$HOME"
git clone https://github.com/mmporong/rby1-tray-lab.git
cd "$HOME/rby1-tray-lab"
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements-lock.txt
.venv/bin/python -m pip install --no-deps -e .
.venv/bin/python -m pip check
.venv/bin/python -c "import mujoco; print(mujoco.__version__)"
```

마지막 출력은 `3.11.0`이어야 한다. `requirements-lock.txt`는 주요 수치·영상 패키지 버전을 고정한다. MuJoCo의 모든 전이 의존성을 고정한 파일은 아니다. MuJoCo 엔진은 Python 패키지에 포함되므로 별도 엔진 설치는 필요 없다. [MuJoCo Python 설치 안내](https://mujoco.readthedocs.io/en/3.11.0/python.html)

### 로봇 모델 받기

```bash
cd "$HOME/rby1-tray-lab"
.venv/bin/python -m rby1_tray.cli fetch-assets
.venv/bin/python -m rby1_tray.cli audit
```

모델은 기본적으로 `$HOME/.cache/rby1-tray-lab/menagerie`에 저장된다. 고정 Menagerie 커밋은 `cda5236f98383f6fbaa4820c6b18dba892cd087f`이며 원격 주소·커밋·Git 작업 상태를 검사한다. 캐시의 XML·mesh·Git 상태를 수정하지 않는다. 캐시 위치를 바꾸려면 **받기와 실행 때 모두** 같은 `RBY1_CACHE`를 지정한다.

## 3. 기본 사이트 실행

Ubuntu 터미널에서 Linux EGL 환경을 지정한다. WSLg처럼 `DISPLAY`가 설정된 환경에서도 아래 값을 명시해 같은 경로로 실행한다.

```bash
cd "$HOME/rby1-tray-lab"
export MUJOCO_GL=egl
export PYOPENGL_PLATFORM=egl
export EGL_PLATFORM=surfaceless
if [ -f /usr/share/glvnd/egl_vendor.d/50_mesa.json ]; then
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json
fi
.venv/bin/python -m rby1_tray.cli view --port 8786
```

실행 터미널을 유지한 채 **실행 PC의 브라우저**에서 <http://127.0.0.1:8786/>를 연다. WSL2이면 Windows 브라우저에서 같은 주소로 접속한다. 종료는 서버 터미널에서 `Ctrl+C`다.

새 clone에는 `artifacts/structural-whole`이 없으므로 전신 응력 화면은 아직 활성화되지 않는다. 원본 영상·관절 부하 영상·RGB/깊이는 저장소에 들어 있다. 화면의 `1배속`은 물리 시간의 8배속이고 `1.5배속`·`2배속`은 각각 12배속·16배속이다. 사이트 재생을 위해 새 물리 실험을 실행할 필요는 없다.

다른 터미널에서 기본 재생 자료가 로드됐는지 확인할 수 있다.

```bash
cd "$HOME/rby1-tray-lab"
.venv/bin/python - <<'PY'
import json
from urllib.request import urlopen
with urlopen("http://127.0.0.1:8786/api/playback", timeout=15) as response:
    data = json.load(response)
print({key: data.get(key) for key in ("enabled", "fps", "frame_count", "base_speed")})
assert data.get("enabled") is True, "재생 자료 검증 실패: 서버 로그를 확인하세요"
PY
```

현재 기록의 정상 출력은 `enabled=True`, `fps=60`, `frame_count=1136`, `base_speed=8`이다. 재생 자료가 없거나 설정·해시가 맞지 않으면 실시간 계산 모드로 전환하므로, 페이지가 열렸다는 사실만으로 재생 자료의 정상 로드를 판단하지 않는다.

개선 연구는 저장소의 `evidence/improvement`를 자동으로 읽는다. 첫 화면의 `개선 전후 검증 보기`를 누르면 경로 비교, 센서 이상 거부, 양팔 협조 비용과 대표 영상을 확인할 수 있다. 이 영상은 상단 원본·센서·응력 타임라인과 별도로 재생된다. 구조 번들 없이도 개선 연구는 표시된다.

```bash
cd "$HOME/rby1-tray-lab"
.venv/bin/python tools/verify_improvement_evidence.py evidence/improvement
```

검증에 실패하면 개선 패널에 `검증 근거 없음`과 사유를 표시한다. 파일을 임의로 섞거나 해시만 갱신하지 말고 같은 커밋의 번들을 다시 받는다. 새 실험·영상·compact 번들을 생성할 때의 도구와 원본 소스 조건은 [개선 연구 재현 절차](improvement-study.md#재현)를 따른다. 기존 개선 영상 재생에는 FFmpeg가 필요하지 않다.

## 4. 현재와 같은 전신 응력·하중 진단 화면 준비

### 필요한 파일

완성된 구조 번들은 약 30 MiB다. 아래 6개를 **같은 번들에서 한 세트로** 가져온다.

```text
$HOME/rby1-tray-lab/artifacts/structural-whole/
├── manifest.json
├── geometry.bin
├── frames.bin
├── fem-surface.bin
├── fem-cutaway.bin
└── diagnostics.bin
```

`diagnostics.bin`은 자중·외력·운동 잔차 진단에 필요하다. `session-handoff.json`, `.venv`, STEP, BREP, FEM 중간 NPZ는 완성된 번들 재생에 필요하지 않다.

이 번들에는 제조사 CAD의 파생 데이터가 포함되어 **공개 재배포 범위를 확인하지 못한 상태**다. 따라서 GitHub 저장소에 포함하지 않는다. 아래 이동 절차는 해당 CAD·파생 데이터의 이용 및 기기 간 이전이 허용된 경우에 사용한다. 허용 범위가 불명확하면 제조사 조건을 확인하고 각 환경에서 CAD를 확보해 재생성한다. Menagerie 모델의 Apache-2.0 라이선스를 제조사 STEP에 적용하지 않는다.

### 기존 PC에서 묶기 → 새 PC에 풀기

기존 PC에서 실행한다. `tar` 출력 파일이 이미 있으면 덮어쓰므로 새 파일명을 사용한다.

```bash
cd "$HOME/rby1-tray-lab"
tar -czf "$HOME/rby1-structural-whole.tar.gz" \
  -C artifacts/structural-whole \
  manifest.json geometry.bin frames.bin fem-surface.bin fem-cutaway.bin diagnostics.bin
```

허용된 전송 방법으로 압축 파일을 새 PC의 **Ubuntu 홈**인 `$HOME/rby1-structural-whole.tar.gz`에 옮긴다. Windows 다운로드 폴더와 WSL의 Linux 홈은 서로 다르다.

새 PC에서 서버를 `Ctrl+C`로 종료한 뒤 실행한다. `mkdir artifacts/structural-whole`이 이미 존재한다는 오류를 내면 기존 번들을 확인한다. 아래 명령은 기존 폴더를 덮어쓰지 않는다.

```bash
cd "$HOME/rby1-tray-lab"
mkdir -p artifacts
mkdir artifacts/structural-whole && \
  tar -xzf "$HOME/rby1-structural-whole.tar.gz" -C artifacts/structural-whole
```

구조 번들과 원본 재생 기록의 해시·프레임·스키마를 함께 검사한다.

```bash
cd "$HOME/rby1-tray-lab"
.venv/bin/python - <<'PY'
from pathlib import Path
from rby1_tray.playback import PlaybackAssets
from rby1_tray.scene import load_config
from rby1_tray.structural_scene import StructuralSceneAssets
playback = PlaybackAssets(Path("evidence/playback-native"), load_config("configs/mobile-native.json"))
scene = StructuralSceneAssets(Path("artifacts/structural-whole"), playback)
info = scene.info()
print({"enabled": info["enabled"], "frames": info["frame_count"],
       "parts": len(info["parts"]), "load_diagnostics": "load_diagnostics" in info})
assert "load_diagnostics" in info, "하중 진단 데이터가 없는 번들입니다"
PY
```

현재 번들은 `enabled=True`, `frames=1136`, `parts=38`, `load_diagnostics=True`로 출력된다. 실패하면 누락된 파일이나 번들·재생 기록의 버전 조합을 확인한다. **검사를 통과시키려고 manifest의 해시를 바꾸지 않는다.**

3절의 EGL 환경이 설정된 터미널에서 다시 실행한다.

```bash
cd "$HOME/rby1-tray-lab"
.venv/bin/python -m rby1_tray.cli view --port 8786 \
  --structural-scene-directory artifacts/structural-whole
```

브라우저를 새로 고침한다. 원본과 `응력 + 관절 부하`가 나란히 표시되고, 재생 시 전신 자세·응력·20관절 부하·하중 진단·RGB/깊이가 기록 시각을 따라 바뀌는지 확인한다. 부품 선택과 CAD 표면/단면 전환도 확인한다. RGB/깊이 모니터는 기록된 1 Hz 관측이므로 60 FPS 영상처럼 매 프레임 바뀌지는 않는다.

번들이 없으면 [제조사 CAD 확보 안내](structural-cad.md)와 [구조 데이터 재생성 절차](structural-analysis.md#재현)를 따른다. 재생성에는 별도 CAE 환경·공식 STEP·중간 해석 결과가 필요하며, 완성된 번들을 재생하는 절차와 다르다. `build_load_diagnostics.py`의 최종 출력 경로를 사용했다면 `--structural-scene-directory`에도 그 경로를 지정한다.

## 5. 기존 PC의 화면을 다른 PC에서 보기

기존 Linux PC의 서버와 구조 번들을 그대로 사용하려면 SSH 터널을 쓸 수 있다. 기존 PC에 SSH 접속 권한과 SSH 서버가 준비되어 있어야 한다. 기존 PC에서 3절 또는 4절의 서버를 실행해 둔다.

**다른 PC의 터미널**에서 아래 예시의 `linux_user`와 `server_ip`를 기존 PC의 실제 계정·주소로 바꾼다. Windows는 OpenSSH가 설치된 PowerShell에서도 실행할 수 있다.

```bash
ssh -N -L 18786:127.0.0.1:8786 linux_user@server_ip
```

다른 PC의 브라우저에서 <http://127.0.0.1:18786/>를 연다. 터널 터미널을 닫거나 `Ctrl+C`를 누르면 연결이 끝난다. 기존 PC의 서버는 별도로 종료해야 한다. 이 방법은 새 PC에 MuJoCo나 CAD를 복사하지 않고 기존 서버의 결과를 보는 방식이다.

현재 CLI는 `127.0.0.1`에만 바인딩하며 `--host` 옵션이 없다. 다른 PC에서 `http://기존PC주소:8786`을 입력하는 방식으로는 접속되지 않는다. 각 PC의 `127.0.0.1`은 그 PC를 뜻한다.

## 6. 다시 실행·업데이트·새 실험

다음 실행에서는 3절의 EGL 설정과 `view` 명령만 다시 실행한다. 구조 번들은 `--structural-scene-directory`로 지정할 수 있고, 기본 위치 `artifacts/structural-whole`에 있으면 자동 탐색한다.

소스를 수정하지 않은 복제본을 업데이트할 때는 다음을 실행한다. 로컬 수정이 있거나 fast-forward가 불가능하면 Git 오류를 확인하고 강제 초기화하지 않는다.

```bash
cd "$HOME/rby1-tray-lab"
git pull --ff-only origin main
.venv/bin/python -m pip install -r requirements-lock.txt
.venv/bin/python -m pip install --no-deps -e .
```

업데이트 후 2절의 `fetch-assets`·`audit`와 3·4절의 검사를 다시 수행한다. 모델 고정 커밋이 바뀌면 기존 캐시를 자동 갱신하지 않고 오류를 낸다. 이때는 새 `RBY1_CACHE` 경로에 모델을 받고 실행할 때도 그 경로를 지정한다. 원본 재생 기록이 바뀌면 이전 구조 번들과 맞지 않을 수 있다. 다른 `--config`를 사용한 실험은 기본 재생 기록과 일치한다고 가정하지 않는다.

새 물리 실험은 별도 출력 폴더로 실행한다. 다음 명령은 사이트를 보기 위한 필수 단계가 아니며, 기존 폴더를 덮어쓰지 않는다.

```bash
cd "$HOME/rby1-tray-lab"
.venv/bin/python -m rby1_tray.cli run --output artifacts/mobile-demo-new --render
```

이 실행 결과가 기본 사이트 재생 자료로 자동 교체되지는 않는다. 영상 번들 생성과 데이터 계약은 [저장소 README](../README.md#실행), [센서·전이 계약](sensors-and-transfer.md)을 참고한다. 영상 번들을 다시 만들 때만 FFmpeg/FFprobe가 필요하다.

## 7. 문제 해결

| 증상 | 확인·조치 |
|---|---|
| `No module named rby1_tray` | `$HOME/rby1-tray-lab`에서 `.venv/bin/python -m pip install --no-deps -e .`를 실행하고 같은 Python으로 실행한다. |
| `python3.12`·`venv`가 없음 | Ubuntu 버전과 `python3.12-venv` 설치를 확인한다. 다른 PC의 가상환경을 복사하지 않는다. |
| `fetch-assets`에서 네트워크 오류 | GitHub 접근·프록시·DNS를 확인한다. 모델 폴더만 임의로 복사하면 Git 출처 검사를 통과하지 못한다. |
| 캐시 출처·커밋·dirty 오류 | 수정한 캐시를 유지해야 하면 다른 새 `RBY1_CACHE` 경로를 지정해 받는다. 실행에도 같은 경로를 지정한다. |
| `EGL_NOT_INITIALIZED`, EGL·OpenGL 오류, 빈 렌더 | 3절 환경과 Mesa 패키지를 확인한다. Linux 소프트웨어 렌더 대안은 아래 OSMesa 절차를 사용한다. |
| 페이지는 열리지만 재생이 느리거나 실시간 모드로 표시됨 | `/api/playback`에서 `enabled`를 확인한다. `evidence/playback-native` 누락·설정 불일치·해시 오류는 서버 로그로 확인한다. |
| 응력 화면이 비활성 | 구조 번들은 Git에 없다. 4절의 6개 파일과 경로를 확인한다. 새 clone에는 고정 CAD 응력 스냅샷도 포함되지 않는다. |
| 구조 번들 해시·스키마·reference states 오류 | 같은 세트의 6개 파일과 대응 재생 기록을 사용한다. 서버를 종료하고 번들을 수정한 뒤 다시 실행한다. |
| 영상은 보이는데 3D만 빈 화면·WebGL 오류 | 브라우저의 WebGL 사용 가능 여부와 그래픽 설정을 확인한다. 서버 EGL과 브라우저 WebGL은 서로 다른 렌더링 경로다. |
| `Address already in use` | 기존 서버를 종료하거나 `view --port 8787`로 실행하고 브라우저도 `http://127.0.0.1:8787/`로 연다. |
| Windows 브라우저에서 WSL 서버에 접속 안 됨 | WSL 터미널에서 서버 실행과 API 응답을 먼저 확인한다. 그 뒤 [Microsoft WSL 네트워크 안내](https://learn.microsoft.com/windows/wsl/networking#accessing-linux-networking-apps-from-windows-localhost)로 localhost 전달 설정을 확인한다. |
| 종료했는데 다시 켜짐 | 이 문서는 터미널 실행 기준이다. 기존 PC에서 별도 서비스로 띄웠다면 해당 서비스도 종료해야 한다. |

**OSMesa 대안은 Linux 전용이며 이 문서 작성 시 실행 검증하지 않았다.** EGL 서버를 종료한 뒤 아래를 시도할 수 있다. 기본 EGL 명령과 섞지 말고 이 환경에서 실행한다. 성공 여부와 성능은 해당 PC에서 확인한다.

```bash
cd "$HOME/rby1-tray-lab"
sudo apt install -y libosmesa6
unset __EGL_VENDOR_LIBRARY_FILENAMES
MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa \
  .venv/bin/python -m rby1_tray.cli view --port 8786
```

## 8. 검증 범위

이 문서의 실행 명령은 Ubuntu 24.04 x86-64, Python 3.12, MuJoCo 3.11.0에서 저장소의 별도 복제본과 새 가상환경으로 확인했다. 같은 호스트의 EGL 드라이버와 검증된 Menagerie 캐시를 재사용했으므로, 다른 물리 PC의 드라이버·새 OS 패키지 설치·WSL2 실행까지 검증한 결과는 아니다. 아래는 서버·파일 검증 결과이며 다른 PC 브라우저의 영상 디코딩·WebGL 렌더링은 해당 환경에서 확인해야 한다.

| 검사 | 결과 |
|---|---|
| 고정 Python 의존성 설치·editable 설치·`pip check` | 통과 · 의존성 오류 없음 |
| 고정 모델 캐시 검증·`audit` | 통과 · RBY1-M 1.2, nq=65, nv=60, nu=28 |
| 구조 번들 없는 새 복제본의 웹 서버·재생 API | 통과 · 60 FPS, 1,136프레임, 구조 비활성 확인, 원본 MP4 응답·960×720 EGL 렌더 확인 |
| 구조 번들 6개 이동·해시/스키마 검사·동기 구조 API | 통과 · 38부품·20관절·하중 진단, 바이너리 5개 HTTP 응답의 원본 일치 확인 |
| 테스트 서버 종료·기존 서버 유지 | 통과 · 테스트 포트 18786 종료, 기존 8786 구조 API 정상 유지 |

이 사이트는 기록된 시뮬레이션과 구조 후처리를 표시한다. 실물 로봇 연결·실측 토크·실제 재료 및 체결 조건을 검증한 결과로 해석하지 않는다.
