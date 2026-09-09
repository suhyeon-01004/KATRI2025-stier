# 소스 가져오기 및 검증 기록

## 원본

- 파일: `src_251031-1725.zip`
- 크기: 397,097,351 bytes
- SHA-256: `d675a7e431e53206548e30f60952de7f42cb6cd95d70361c9badf05ff800dcc7`
- 대상: `suhyeon01004-hongik/Katri-2025`, `main`
- 사용자 확인 역할: 정적장애물 회피

## 가져오기 정책

압축을 풀어 `src/` 구조를 유지하고 원본 ZIP은 변경하지 않았습니다. 38개 ROS 패키지의 `package.xml`을 모두 포함했습니다. 각 하위 프로젝트의 기존 ignore 규칙을 존중하며, 중첩 Git 저장소 이력·중복 ZIP·빌드·캐시·컴파일 산출물·과거 주행 로그는 Git 관리에서 제외했습니다. RDDF 경로 파일과 기존 라이선스·문서는 보존했습니다.

Git 인덱스에는 원본 ZIP에 기록된 실행 권한 및 catkin 심볼릭 링크 형식을 복원했습니다. Windows 체크아웃에서 링크가 일반 파일로 보일 수 있으며, Linux에서는 README의 catkin 링크 재생성 절차를 참고합니다.

`src/recognition/src/lane/69.pth` (262,970,373 bytes)는 Git LFS 대상으로 포함했습니다. 차선 인식의 기존 Python 3.8/Linux 네이티브 확장 바이너리는 제외하므로 사용 환경에서 다시 빌드해야 합니다.

## 소스 수정 범위

다음 세 launch 파일의 NTRIP 계정 기본값만 `$(optenv NTRIP_USERNAME)`, `$(optenv NTRIP_PASSWORD)`로 바꿨습니다.

- `src/control/gps_bringup/launch/gps.launch`
- `src/control/ublox_utils/launch/ublox.launch`
- `src/Xsens_MTi_ROS_Driver_and_Ntrip_Client/src/ntrip/launch/ntrip.launch`

원본 접속값은 문서나 커밋 메시지에 기록하지 않았습니다. README, 이 기록, 루트 ignore 규칙 및 LFS 속성을 추가했으며 주행 알고리즘은 수정하지 않았습니다.

## 수행한 검증

| 항목 | 결과 |
| --- | --- |
| ROS 패키지 매니페스트 | 38개 XML 읽기 성공, 이름 중복 없음, 전부 Git에 포함 |
| 핵심 Python 문법 | `avoid_static_offset.py`, `avoid_static_large.py`, `pure_pursuit_lidar.py` AST 파싱 성공 |
| 핵심 소스 보존 | 위 3개 파일의 로컬 바이트가 ZIP 원본과 일치 |
| 수정한 launch | 3개 XML 파싱 성공 |
| README 소스 링크 | 참조한 로컬 소스 파일 존재 확인 |
| 업로드 대상 확인 | 스테이징된 3,608개 파일에서 원본 계정 리터럴 및 주요 토큰·개인키 패턴 일치 없음; 이 보고서는 그 후 추가 |
| LFS | `69.pth`가 LFS 포인터로 스테이징됨 |

문법·구조 검사는 실행 성능 검증을 대체하지 않습니다. ROS/catkin 전체 빌드, CUDA 확장 컴파일, 센서 재생, 실제 차량 주행은 이 Windows 환경에서 수행하지 않았습니다. README의 복원 과제 및 경로 생성 한계를 함께 확인해야 합니다.
