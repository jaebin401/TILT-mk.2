# TILT Git Convention

## 1. Branch Strategy

TILT는 다음 브랜치 구조를 사용한다.

```text
feat/*  ──┐
fix/*   ──┤
docs/*  ──┼── Pull Request ──> dev ── Release PR ──> main
chore/* ──┤
test/*  ──┘
```

### main

조립, 시연 또는 배포할 수 있는 안정 상태를 보관한다.

- 직접 Push하지 않는다.
- `dev`에서 생성한 Pull Request만 병합한다.
- 펌웨어 빌드와 필요한 하드웨어 검증을 통과해야 한다.
- TILT MK 버전 또는 주요 마일스톤 단위로 갱신한다.
- Force Push하지 않는다.

### dev

다음 안정 버전을 준비하는 통합 브랜치다.

- 모든 작업 브랜치는 최신 `dev`에서 생성한다.
- 작업 브랜치의 Pull Request를 통해 변경을 반영한다.
- 직접 Push와 Force Push를 지양한다.
- 통합 후 정상 동작이 확인되면 `main`으로 Release PR을 생성한다.

### 작업 브랜치

| Prefix | 용도 | 예시 |
| --- | --- | --- |
| `feat/` | 새로운 기능이나 기구 추가 | `feat/servo-zero-position` |
| `fix/` | 기존 동작 또는 설계 오류 수정 | `fix/servo-id-conflict` |
| `docs/` | 문서 작업 | `docs/assembly-guide` |
| `chore/` | 저장소 설정과 개발 환경 | `chore/platformio-setup` |
| `test/` | 하드웨어, 제어 방식 또는 설계 실험 | `test/gait-sequence` |

브랜치 이름은 영문 소문자와 하이픈을 사용한다.

한 브랜치에서는 하나의 명확한 작업만 진행한다.

`test/*` 브랜치는 실험 결과를 검증하고 정리하기 위한 브랜치다. 실험 결과를 제품 코드에 반영하려면 불필요한 임시 코드를 제거한 뒤 `dev`를 대상으로 Pull Request를 생성한다. 폐기한 실험은 결과와 판단 근거만 문서로 남길 수 있다.

## 2. Standard Workflow

작업을 시작하기 전에 최신 `dev`를 가져온다.

```bash
git switch dev
git pull --ff-only origin dev
git switch -c feat/servo-zero-position
```

작업 후 원격 저장소에 Push한다.

```bash
git push -u origin feat/servo-zero-position
```

Pull Request 방향은 다음과 같다.

```text
base: dev
compare: feat/servo-zero-position
```

병합된 작업 브랜치는 삭제한다.

## 3. Commit Convention

기본 형식:

```text
<type>(<scope>): <subject>
```

예시:

```text
feat(firmware): add servo ping command
feat(cad): add MK1 hip bracket
fix(firmware): prevent duplicate servo IDs
docs(hardware): document servo wiring
chore(repo): configure initial repository
```

### Type

| Type | 의미 |
| --- | --- |
| `feat` | 새로운 기능, 동작 또는 기구 추가 |
| `fix` | 잘못된 동작이나 설계 수정 |
| `refactor` | 동작 변화 없는 코드 구조 개선 |
| `docs` | 문서 추가 및 수정 |
| `test` | 테스트 추가 및 수정 |
| `chore` | 저장소, 빌드, 개발 환경 설정 |
| `revert` | 기존 변경 되돌리기 |

### Scope

| Scope | 적용 대상 |
| --- | --- |
| `firmware` | ESP32-S3 펌웨어 |
| `cad` | 기구 설계 원본과 출력 파일 |
| `hardware` | 배선, 전원, BOM |
| `tools` | 서보 설정과 보행 개발 도구 |
| `docs` | 일반 문서 |
| `repo` | 저장소 전체 설정과 구조 |

### Subject 규칙

- 영어로 작성한다.
- 소문자로 시작한다.
- 동사 원형으로 시작한다.
- 문장 끝에 마침표를 사용하지 않는다.
- 가능하면 72자 이내로 작성한다.
- `update`, `change`, `work`처럼 모호한 표현만 사용하지 않는다.
- 하나의 Commit에는 하나의 논리적 변경만 포함한다.

## 4. Pull Request

Pull Request 제목도 Commit과 같은 형식을 사용한다.

```text
feat(cad): add MK1 hip bracket
fix(firmware): prevent servo overtravel
```

본문에는 다음 내용을 작성한다.

```markdown
## 작업 내용

- 무엇을 구현하거나 수정했는지 작성한다.

## 변경사항

- 주요 파일과 구조 변경을 작성한다.

## 확인 사항

- [ ] 관련 빌드 또는 파일 열기 성공
- [ ] 실제 하드웨어 또는 CAD 동작 확인
- [ ] 관련 없는 파일이 포함되지 않음
- [ ] 임시 파일과 빌드 결과물이 포함되지 않음

## 참고 자료

- 사진, 영상, 관련 문서 또는 이슈
- 없다면 `없음`
```

가능하면 다른 작업자 한 명이 검토한 후 병합한다.

작업 브랜치에서 `dev`로 병합할 때는 `Squash and merge`를 사용한다.

`dev`에서 `main`으로 승격할 때는 `Create a merge commit`을 사용한다.


## 5. Verification

### Firmware

- 프로젝트가 정상적으로 빌드되는가
- 서보 ID와 보드 설정이 올바른가
- 관절 제한을 벗어나는 명령이 없는가
- 실제 장비 테스트 결과를 PR에 기록했는가

### CAD

- 원본 파일이 정상적으로 열리는가
- STEP 또는 3MF로 정상적으로 내보낼 수 있는가
- 부품 간 간섭을 확인했는가
- 출력 방향과 revision을 확인했는가

### Repository

- 빌드 결과물과 개인 설정이 포함되지 않았는가
- `.DS_Store`와 임시 파일이 포함되지 않았는가
- 비밀번호, 토큰 또는 개인 환경 설정이 포함되지 않았는가
