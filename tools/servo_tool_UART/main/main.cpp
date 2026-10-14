// TILT — 서보 셋업 / 캘리브레이션 툴
//
// 목적: tilt_config.h 의 TODO 항목(ZERO_TICK, JOINT_SIGN, JOINT_LIMIT)을
//       실측으로 채우는 것. 모든 결과는 그대로 붙여넣을 수 있는 C 코드로 출력한다.
//
// 관절: config 의 8관절 (LHR LHP LKP LAP | RHR RHP RKP RAP), 다리당 NUM_LEG_JOINTS = 4.
//
// 저장 정책:
//   - 진실 공급원은 tilt_config.h 하나뿐이다.
//   - NVS 는 "작업 중 날아가지 않게" 하는 임시 스크래치일 뿐이며,
//     캘리브레이션이 끝나면 반드시 출력값을 config 에 붙여넣고 NVS 는 지운다.
//
// 빌드:  idf.py set-target esp32s3 && idf.py -p <PORT> flash monitor

#include <cstdio>
#include <cstring>
#include <cctype>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "tilt_config.h"
#include "tilt/sts3215/Sts3215Bus.h"

using namespace tilt;

// ─────────────────────────────────────────────────────────────────────
// Existing tool calls are kept behind a thin adapter to the current
// tilt::sts3215::Sts3215Bus API.
// ─────────────────────────────────────────────────────────────────────

static tilt::sts3215::Sts3215Bus& bus() {
    static tilt::sts3215::Sts3215Bus instance{[] {
        tilt::sts3215::BusConfig cfg{};
        cfg.uart_port = static_cast<uart_port_t>(tilt::SERVO_UART_PORT);
        cfg.tx_pin = static_cast<gpio_num_t>(tilt::SERVO_UART_TX_PIN);
        cfg.rx_pin = static_cast<gpio_num_t>(tilt::SERVO_UART_RX_PIN);
        cfg.baud_rate = tilt::SERVO_UART_BAUD;
        return cfg;
    }()};
    return instance;
}

static bool servo_bus_init() {
    return bus().initialize() == ESP_OK;
}

static bool servo_ping(uint8_t id) {
    return bus().ping(id) == ESP_OK;
}

static bool servo_read_tick(uint8_t id, uint16_t* out_tick) {
    return out_tick != nullptr && bus().readPosition(id, *out_tick) == ESP_OK;
}

static void servo_write_tick(uint8_t id, uint16_t tick, uint16_t speed) {
    bus().writePosition(id, tick, speed);
}

static bool servo_torque(uint8_t id, bool on) {
    return bus().setTorque(id, on) == ESP_OK;
}

static void servo_set_baud(uint32_t baud) {
    bus().setBaudRate(baud);
}

static bool servo_read_byte(uint8_t id, uint8_t address, uint8_t* value) {
    return value != nullptr && bus().readRegister(id, address, value, 1) == ESP_OK;
}

static bool servo_write_byte(uint8_t id, uint8_t address, uint8_t value) {
    return bus().writeRegister(id, address, &value, 1) == ESP_OK;
}

static constexpr uint8_t STS3215_ID_ADDRESS = 5;
static constexpr uint8_t STS3215_EEPROM_LOCK_ADDRESS = 55;
static constexpr uint8_t STS3215_EEPROM_UNLOCKED = 0;
static constexpr uint8_t STS3215_EEPROM_LOCKED = 1;
static constexpr uint8_t SERVO_ID_MIN = 0;
static constexpr uint8_t SERVO_ID_MAX = 253;

static_assert(NUM_JOINTS == 8 && NUM_LEG_JOINTS == 4, "servo tool expects 2 legs x 4 joints");

static bool is_ankle(int j) { return j == L_ANKLE_PITCH || j == R_ANKLE_PITCH; }

// 8관절 형식은 이전(6관절) 스크래치와 크기가 달라서 키를 새로 쓴다.
static const char *NVS_NAMESPACE = "tilt_cal";
static const char *NVS_KEY_ZERO  = "zero_tick8";
static const char *NVS_KEY_SIGN  = "joint_sign8";

// ── 작업용 상태 ──────────────────────────────────────────────────────
// config 값으로 초기화한 뒤, NVS 스크래치가 있으면 덮어쓴다.
static uint16_t s_zero[NUM_JOINTS];
static int8_t   s_sign[NUM_JOINTS];

// 조그 중 추적하는 목표 위치(절대 점프 방지용)
static uint16_t s_cur[NUM_JOINTS];

// 관절 제한 [deg] — [j][0] = 최소(−쪽), [j][1] = 최대(+쪽). config JOINT_LIMIT 값으로 시작.
static float s_lim_deg[NUM_JOINTS][2];
static bool  s_lim_new[NUM_JOINTS][2];   // 이번 세션에서 새로 확정했는지

static bool s_coarse = false;   // false = FINE(1 tick), true = COARSE
static bool s_boot_torque_on = false;  // 부팅 시 토크 ON 성공 여부 (메뉴 t 토글 초기값)

static constexpr int FINE_STEP   = 1;
static constexpr int COARSE_STEP = 20;

static const char *JOINT_LABEL[NUM_JOINTS] = {
    "L_HIP_ROLL", "L_HIP_PITCH", "L_KNEE_PITCH", "L_ANKLE_PITCH",
    "R_HIP_ROLL", "R_HIP_PITCH", "R_KNEE_PITCH", "R_ANKLE_PITCH",
};

static const char *DH_PLUS_MEANING[NUM_JOINTS] = {
    "발이 왼쪽(바깥)으로", "다리가 뒤로 스윙", "무릎이 굽음", "발끝이 내려감 (toes down)",
    "발이 왼쪽(안쪽)으로", "다리가 뒤로 스윙", "무릎이 굽음", "발끝이 내려감 (toes down)",
};

static int   step_size()  { return s_coarse ? COARSE_STEP : FINE_STEP; }
static const char *mode_name() { return s_coarse ? "COARSE" : "FINE"; }

// ── 단위 변환 (config 의 상수가 아니라 작업용 s_zero/s_sign 사용) ────
static float tick_to_dh_rad(int j, uint16_t tick) {
    return (static_cast<int>(tick) - static_cast<int>(s_zero[j]))
           * RAD_PER_TICK * s_sign[j];
}

static float tick_to_dh_deg(int j, uint16_t tick) {
    return tick_to_dh_rad(j, tick) * 180.0f / 3.14159265f;
}

static uint16_t dh_rad_to_tick(int j, float rad) {
    float  t   = rad / RAD_PER_TICK * s_sign[j];
    int    raw = static_cast<int>(s_zero[j] + t + 0.5f);
    if (raw < SERVO_POS_MIN) raw = SERVO_POS_MIN;
    if (raw > SERVO_POS_MAX) raw = SERVO_POS_MAX;
    return static_cast<uint16_t>(raw);
}

static uint16_t clamp_tick(int32_t v) {
    if (v < SERVO_POS_MIN) return SERVO_POS_MIN;
    if (v > SERVO_POS_MAX) return SERVO_POS_MAX;
    return static_cast<uint16_t>(v);
}

// 관절이 지금 pose_rad 자세에 있을 때의 tick 으로부터 영점 tick 을 역산한다.
// (pose_rad = 0 이면 현재 tick 이 곧 영점)
static uint16_t zero_from_pose(int j, uint16_t tick, float pose_rad) {
    const float offset = pose_rad / RAD_PER_TICK * s_sign[j];
    return clamp_tick(static_cast<int32_t>(lroundf(static_cast<float>(tick) - offset)));
}

// ── NVS 스크래치 ─────────────────────────────────────────────────────
static void nvs_init_once() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
}

// apply = false: 부팅 시 — config 와 다른 스크래치가 있는지 알리기만 한다.
// apply = true : 메뉴 r — 스크래치를 작업값으로 불러온다.
// (예전에는 부팅 때 자동으로 덮어써서, config 를 고쳐도 툴은 옛 영점을 쓰는 문제가 있었다)
static void nvs_load_scratch(bool apply) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        if (apply) printf(">> NVS 스크래치가 없습니다.\n");
        return;
    }

    uint16_t zero[NUM_JOINTS];
    int8_t sign[NUM_JOINTS];
    size_t sz = sizeof(zero);
    const bool has_zero = nvs_get_blob(h, NVS_KEY_ZERO, zero, &sz) == ESP_OK && sz == sizeof(zero);
    sz = sizeof(sign);
    const bool has_sign = nvs_get_blob(h, NVS_KEY_SIGN, sign, &sz) == ESP_OK && sz == sizeof(sign);
    nvs_close(h);

    if (!has_zero && !has_sign) {
        if (apply) printf(">> NVS 스크래치가 없습니다.\n");
        return;
    }
    if (apply) {
        if (has_zero) memcpy(s_zero, zero, sizeof(zero));
        if (has_sign) memcpy(s_sign, sign, sizeof(sign));
        printf(">> NVS 스크래치를 작업값으로 불러왔습니다 (config 값이 아님).\n");
        printf("   h / u 이동은 여전히 tilt_config.h 값을 기준으로 합니다.\n");
        return;
    }
    const bool differs = (has_zero && memcmp(zero, ZERO_TICK, sizeof(zero)) != 0) ||
                         (has_sign && memcmp(sign, JOINT_SIGN, sizeof(sign)) != 0);
    if (differs) {
        printf("[NVS] tilt_config.h 와 다른 임시 저장값이 있습니다 — 지금은 config 값을 씁니다.\n");
        printf("      이어서 작업하려면 r (스크래치 불러오기), 필요 없으면 c (비우기).\n");
    }
}

static void nvs_save_scratch() {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        printf("[NVS] 저장 실패 — 출력된 값을 꼭 config 에 붙여넣으세요.\n");
        return;
    }
    nvs_set_blob(h, NVS_KEY_ZERO, s_zero, sizeof(s_zero));
    nvs_set_blob(h, NVS_KEY_SIGN, s_sign, sizeof(s_sign));
    nvs_commit(h);
    nvs_close(h);
}

static void nvs_clear_scratch() {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
    printf(">> NVS 스크래치를 비웠습니다.\n");
}

// ── 출력: 붙여넣기용 C 코드 ──────────────────────────────────────────
static void print_zero_array() {
    printf("\n// ── tilt_config.h 의 ZERO_TICK 을 아래로 교체 ──\n");
    printf("inline constexpr std::uint16_t ZERO_TICK[NUM_JOINTS] = {\n");
    for (int j = 0; j < NUM_JOINTS; ++j) {
        printf("    %4u,  // %s\n", s_zero[j], JOINT_LABEL[j]);
    }
    printf("};\n\n");
}

static void print_sign_array() {
    printf("\n// ── tilt_config.h 의 JOINT_SIGN 을 아래로 교체 ──\n");
    printf("inline constexpr std::int8_t JOINT_SIGN[NUM_JOINTS] = {\n");
    for (int j = 0; j < NUM_JOINTS; ++j) {
        printf("    %+d,  // %s\n", s_sign[j], JOINT_LABEL[j]);
    }
    printf("};\n\n");
}

static constexpr float RAD2DEG = 57.2957795f;

static void init_limits_from_config() {
    for (int j = 0; j < NUM_JOINTS; ++j) {
        const JointLimit& lim = JOINT_LIMIT[j / NUM_LEG_JOINTS][j % NUM_LEG_JOINTS];
        s_lim_deg[j][0] = lim.minimum_rad * RAD2DEG;
        s_lim_deg[j][1] = lim.maximum_rad * RAD2DEG;
        s_lim_new[j][0] = s_lim_new[j][1] = false;
    }
}

static void print_limit_array() {
    printf("\n// ── tilt_config.h 의 JOINT_LIMIT 을 아래로 교체 ──\n");
    printf("//    (실측 = 이번에 확정한 값, 기존 = config 값 유지)\n");
    printf("inline constexpr JointLimit JOINT_LIMIT[2][NUM_LEG_JOINTS] = {\n");
    for (int leg = 0; leg < 2; ++leg) {
        printf("    {\n");
        for (int leg_joint = 0; leg_joint < NUM_LEG_JOINTS; ++leg_joint) {
            const int j = leg * NUM_LEG_JOINTS + leg_joint;
            const bool a = s_lim_new[j][0], b = s_lim_new[j][1];
            // JointLimit = {minimum, maximum}: always print the smaller value first.
            const float lo = fminf(s_lim_deg[j][0], s_lim_deg[j][1]);
            const float hi = fmaxf(s_lim_deg[j][0], s_lim_deg[j][1]);
            printf("        { %+7.1ff * DEG2RAD, %+7.1ff * DEG2RAD },  // %s (%s)\n",
                   lo, hi, JOINT_LABEL[j],
                   (a && b) ? "실측" : (a || b) ? (a ? "min 실측, max 기존" : "min 기존, max 실측")
                                              : "기존");
        }
        printf("    },\n");
    }
    printf("};\n\n");
}

// ── 공통 동작 ────────────────────────────────────────────────────────
static void torque_all(bool on) {
    for (int j = 0; j < NUM_JOINTS; ++j) {
        servo_torque(SERVO_ID[j], on);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void emergency_stop() {
    bus().emergencyStop(tilt::SERVO_ID, tilt::NUM_JOINTS);
    printf("\n!! 비상정지: 전체 토크 OFF !!\n");
}

static bool read_all(uint16_t out[NUM_JOINTS]) {
    bool ok = true;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        uint16_t t;
        if (servo_read_tick(SERVO_ID[j], &t)) {
            out[j] = t;
        } else {
            ok = false;
        }
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    return ok;
}

// 현재 위치를 goal로 먼저 기록한 뒤 토크 ON — 이전 goal로 튀는 것 방지.
// (예: `o`에서 토크를 끄고 손으로 옮긴 뒤 토크를 켜면, goal이 예전 값이라 튀어 돌아갈 수 있음)
static bool hold_and_torque_on() {
    if (!read_all(s_cur)) {
        printf("  위치 읽기 실패 — 토크를 켜지 않습니다. 배선/전원 확인.\n");
        return false;
    }
    for (int j = 0; j < NUM_JOINTS; ++j) {
        servo_write_tick(SERVO_ID[j], s_cur[j], 0);  // 현재 위치 = 목표 → 움직이지 않음
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    torque_all(true);
    return true;
}

static int read_key_blocking() {
    uint8_t ch;
    while (true) {
        int n = usb_serial_jtag_read_bytes(&ch, 1, pdMS_TO_TICKS(100));
        if (n > 0) return ch;
    }
}

static int read_key_timeout(int ms) {
    uint8_t ch;
    int n = usb_serial_jtag_read_bytes(&ch, 1, pdMS_TO_TICKS(ms));
    return (n > 0) ? ch : -1;
}

// USB Serial/JTAG monitor is character based, so read and echo a small decimal
// line instead of relying on scanf buffering. Returns false when cancelled.
static bool read_servo_id(const char* label, uint8_t* out_id) {
    if (out_id == nullptr) return false;

    while (true) {
        printf("  %s (0~253, x=취소): ", label);
        fflush(stdout);

        int value = 0;
        int digits = 0;
        while (true) {
            const int c = read_key_blocking();

            if (c == ' ' && digits == 0) {
                emergency_stop();
                return false;
            }
            if ((c == 'x' || c == 'X') && digits == 0) {
                printf("x\n");
                return false;
            }
            if ((c == '\b' || c == 0x7f) && digits > 0) {
                value /= 10;
                --digits;
                printf("\b \b");
                fflush(stdout);
                continue;
            }
            if (c >= '0' && c <= '9' && digits < 3) {
                value = value * 10 + (c - '0');
                ++digits;
                putchar(c);
                fflush(stdout);
                continue;
            }
            if (c == '\r' || c == '\n') {
                // Some monitors send CRLF. Ignore the leftover terminator at
                // the beginning of the next prompt.
                if (digits == 0) continue;
                printf("\n");
                if (value >= SERVO_ID_MIN && value <= SERVO_ID_MAX) {
                    *out_id = static_cast<uint8_t>(value);
                    return true;
                }
                printf("  잘못된 ID입니다. 0~253 범위로 다시 입력하세요.\n");
                break;
            }
        }
    }
}

// ── 툴 1: ID 스캔 ────────────────────────────────────────────────────
static void tool_scan_ids() {
    printf("\n=== ID 스캔 (0~253) ===\n");
    int found = 0;
    for (int id = 0; id <= 253; ++id) {
        if (servo_ping(static_cast<uint8_t>(id))) {
            printf("  응답: ID %d\n", id);
            found++;
        }
        vTaskDelay(pdMS_TO_TICKS(4));
    }
    printf("=== 완료: %d개 ===\n", found);
    if (found == 0) {
        printf("  하나도 응답하지 않음 — 전원 / 배선 / 보레이트를 확인하세요.\n");
        return;
    }
    int missing = 0;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (!servo_ping(SERVO_ID[j])) {
            printf("  !! %s (ID %u) 응답 없음\n", JOINT_LABEL[j], SERVO_ID[j]);
            missing++;
        }
        vTaskDelay(pdMS_TO_TICKS(4));
    }
    if (missing == 0) printf("  config 의 %d개 관절 ID 모두 응답.\n", NUM_JOINTS);
}

// ── 툴 2: ID 변경 ───────────────────────────────────────────────────
// STS3215 ID is EEPROM-backed. Keep this sequence aligned with the host-side
// tools/servo_setup implementation: torque off -> unlock -> write -> re-lock.
// Returns true once torque has been disabled, so the menu can track that state.
static bool tool_change_id() {
    printf("\n=== 서보 ID 변경 ===\n");
    printf("  반드시 대상 STS3215 한 개만 버스에 연결하세요.\n");
    printf("  같은 현재 ID를 가진 서보가 여러 개면 모두 함께 변경됩니다.\n");
    printf("  변경 중에는 서보 전원을 끄거나 케이블을 분리하지 마세요.\n\n");

    uint8_t current_id = 0;
    uint8_t new_id = 0;
    if (!read_servo_id("현재 ID", &current_id) ||
        !read_servo_id("새 ID", &new_id)) {
        printf(">> ID 변경 취소\n");
        return false;
    }
    if (current_id == new_id) {
        printf(">> 현재 ID와 새 ID가 같습니다. 변경하지 않습니다.\n");
        return false;
    }
    if (!servo_ping(current_id)) {
        printf(">> 현재 ID %u가 응답하지 않습니다. 전원/배선/보레이트를 확인하세요.\n",
               current_id);
        return false;
    }
    if (servo_ping(new_id)) {
        printf(">> 새 ID %u가 이미 응답합니다. 사용하지 않은 ID를 선택하세요.\n",
               new_id);
        return false;
    }

    printf("\n  ID %u -> %u 로 변경합니다. 계속하려면 y, 취소는 아무 키나: ",
           current_id, new_id);
    fflush(stdout);
    const int confirm = tolower(read_key_blocking());
    printf("%c\n", confirm);
    if (confirm != 'y') {
        printf(">> ID 변경 취소 — EEPROM은 건드리지 않았습니다.\n");
        return false;
    }

    if (!servo_torque(current_id, false)) {
        printf(">> 토크 해제 명령 전송 실패. ID를 변경하지 않습니다.\n");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    if (!servo_write_byte(current_id,
                          STS3215_EEPROM_LOCK_ADDRESS,
                          STS3215_EEPROM_UNLOCKED)) {
        printf(">> EEPROM 잠금 해제 명령 전송 실패. ID를 변경하지 않습니다.\n");
        return true;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    if (!servo_write_byte(current_id, STS3215_ID_ADDRESS, new_id)) {
        servo_write_byte(current_id,
                         STS3215_EEPROM_LOCK_ADDRESS,
                         STS3215_EEPROM_LOCKED);
        printf(">> ID 기록 명령 전송 실패. EEPROM 재잠금을 시도했습니다.\n");
        return true;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    const bool lock_sent = servo_write_byte(new_id,
                                             STS3215_EEPROM_LOCK_ADDRESS,
                                             STS3215_EEPROM_LOCKED);
    vTaskDelay(pdMS_TO_TICKS(20));

    const bool new_responds = servo_ping(new_id);
    const bool old_responds = servo_ping(current_id);
    uint8_t lock_value = 0xff;
    const bool lock_read = new_responds &&
                           servo_read_byte(new_id,
                                           STS3215_EEPROM_LOCK_ADDRESS,
                                           &lock_value);

    if (new_responds && !old_responds && lock_sent && lock_read &&
        lock_value == STS3215_EEPROM_LOCKED) {
        printf("\n>> 서보 ID 변경 성공: %u -> %u\n", current_id, new_id);
        printf("   서보와 케이블에 ID %u를 표시하고, tilt_config.h의 SERVO_ID도 맞추세요.\n",
               new_id);
        return true;
    }

    if (new_responds && !old_responds) {
        printf("\n!! ID는 %u로 변경됐지만 EEPROM 재잠금을 확인하지 못했습니다.\n",
               new_id);
        printf("   서보 전원을 끄고 한 개만 연결된 상태인지 확인한 뒤 다시 시도하세요.\n");
        return true;
    }

    // The write may have failed before or after the ID took effect. Try both
    // addresses so an unlocked EEPROM is not silently left behind.
    servo_write_byte(new_id,
                     STS3215_EEPROM_LOCK_ADDRESS,
                     STS3215_EEPROM_LOCKED);
    servo_write_byte(current_id,
                     STS3215_EEPROM_LOCK_ADDRESS,
                     STS3215_EEPROM_LOCKED);
    printf("\n!! ID 변경 검증 실패 (이전 ID 응답=%s, 새 ID 응답=%s).\n",
           old_responds ? "있음" : "없음",
           new_responds ? "있음" : "없음");
    printf("   전원을 끄고 대상 서보 한 개만 연결한 뒤 p로 현재 ID를 확인하세요.\n");
    return true;
}

// ── 툴 3: 보레이트 스캔 ──────────────────────────────────────────────
static void tool_scan_baud() {
    static const uint32_t CAND[] = {
        1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400, 19200, 9600
    };
    printf("\n=== 보레이트 스캔 ===\n");
    for (uint32_t baud : CAND) {
        printf("  %u bps ...\n", static_cast<unsigned>(baud));
        servo_set_baud(baud);
        vTaskDelay(pdMS_TO_TICKS(20));

        int found = 0;
        for (int j = 0; j < NUM_JOINTS; ++j) {
            if (servo_ping(SERVO_ID[j])) found++;
            vTaskDelay(pdMS_TO_TICKS(4));
        }
        if (found > 0) {
            printf("=== %u bps 에서 %d개 응답. 이 값을 유지합니다. ===\n",
                   static_cast<unsigned>(baud), found);
            printf("    tilt_config.h 의 SERVO_UART_BAUD 를 %u 로 고치세요.\n\n",
                   static_cast<unsigned>(baud));
            return;
        }
    }
    printf("=== 전부 실패. 원래 값(%u bps)으로 복귀 ===\n",
           static_cast<unsigned>(SERVO_UART_BAUD));
    printf("보레이트 문제가 아니라 배선 / 전원 쪽을 의심하세요.\n\n");
    servo_set_baud(SERVO_UART_BAUD);
}

// ── 툴 4: 정밀 조그 (영점 잡기 본체) ─────────────────────────────────
struct KeyMap { char key; int joint; int8_t dir; };
static const KeyMap JOG_KEYS[16] = {
    {'q', L_HIP_ROLL, +1}, {'a', L_HIP_ROLL, -1},
    {'w', L_HIP_PITCH, +1}, {'s', L_HIP_PITCH, -1},
    {'e', L_KNEE_PITCH, +1}, {'d', L_KNEE_PITCH, -1},
    {'u', L_ANKLE_PITCH, +1}, {'j', L_ANKLE_PITCH, -1},
    {'r', R_HIP_ROLL, +1}, {'f', R_HIP_ROLL, -1},
    {'t', R_HIP_PITCH, +1}, {'g', R_HIP_PITCH, -1},
    {'y', R_KNEE_PITCH, +1}, {'h', R_KNEE_PITCH, -1},
    {'o', R_ANKLE_PITCH, +1}, {'l', R_ANKLE_PITCH, -1},
};

// 다리 6관절 영점 확정 (다리를 쭉 편 자세에서). 발목은 건드리지 않는다.
static void set_leg_zero(const uint16_t pos[NUM_JOINTS]) {
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (!is_ankle(j)) s_zero[j] = pos[j];
    }
}

// 발목 영점 확정. hip/knee 가 zero pose 이고 발바닥이 바닥과 평행한 상태여야 한다.
static void set_ankle_zero(const uint16_t pos[NUM_JOINTS]) {
    bool off = false;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (is_ankle(j)) continue;
        const float err = tick_to_dh_deg(j, pos[j]) - ZERO_POSE_RAD[j] * 180.0f / 3.14159265f;
        if (fabsf(err) > 2.0f) {
            printf("  ⚠ %s 가 zero pose 에서 %+.1f deg 벗어나 있음\n", JOINT_LABEL[j], err);
            off = true;
        }
    }
    if (off) {
        printf("  ⚠ hip/knee 가 zero pose 가 아니면 발목 영점이 틀어집니다. u 또는 k 로 자세를 맞춘 뒤 다시 하세요.\n");
    }
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (is_ankle(j)) s_zero[j] = zero_from_pose(j, pos[j], ZERO_POSE_RAD[j]);
    }
}

static void jog_print_help() {
    printf("\n── 정밀 조그 ──────────────────────────────────\n");
    printf("  q/a  w/s  e/d  u/j : 왼쪽  ROLL / PITCH / KNEE / ANKLE  (+/-)\n");
    printf("  r/f  t/g  y/h  o/l : 오른쪽 ROLL / PITCH / KNEE / ANKLE  (+/-)\n");
    printf("  m : FINE(%d tick) <-> COARSE(%d tick) 토글\n", FINE_STEP, COARSE_STEP);
    printf("  i : 8축 현재 상태 표 출력\n");
    printf("  z : 다리 6관절(ROLL/PITCH/KNEE) 영점 확정 — 다리 편 자세에서\n");
    printf("  n : 발목 2관절 영점 확정 — zero pose + 발바닥이 바닥과 평행할 때\n");
    printf("  x : 메뉴로\n");
    printf("  현재 모드: %s (%.3f deg/step)\n\n",
           mode_name(), step_size() * DEG_PER_TICK);
    printf("  ※ 다리: DH theta 가 0.00 에 수렴하도록 맞추세요.\n");
    printf("    ROLL=다리 수직(좌우 안 기울게) / PITCH=허벅지 수직 / KNEE=정강이 일직선\n");
    printf("  ※ 발목: hip/knee 를 zero pose 로 둔 채(u 또는 k) 발바닥이 바닥과 평행하게 맞추고 n\n\n");
}

static void jog_print_table() {
    printf("\n  %-14s %6s %10s %10s\n", "joint", "tick", "raw deg", "DH theta");
    printf("  ------------------------------------------------\n");
    for (int j = 0; j < NUM_JOINTS; ++j) {
        printf("  %-14s %6u %9.2f %9.2f\n",
               JOINT_LABEL[j], s_cur[j],
               s_cur[j] * DEG_PER_TICK,
               tick_to_dh_deg(j, s_cur[j]));
    }
    printf("\n");
}

static void tool_jog() {
    // 현재 위치를 goal로 기록한 뒤 시작 (절대 점프 방지)
    if (!hold_and_torque_on()) return;
    jog_print_help();

    while (true) {
        int c = read_key_blocking();
        if (c == ' ') { emergency_stop(); continue; }

        char key = static_cast<char>(tolower(c));
        if (key == 'x') { printf(">> 메뉴로\n"); return; }
        if (key == 'm') {
            s_coarse = !s_coarse;
            printf(">> %s 모드 (%.3f deg/step)\n", mode_name(), step_size() * DEG_PER_TICK);
            continue;
        }
        if (key == 'i') { jog_print_table(); continue; }
        if (key == 'z') {
            set_leg_zero(s_cur);
            print_zero_array();
            nvs_save_scratch();
            printf(">> 다리 6관절 영점 확정. NVS 에 임시 저장했습니다. 위 배열을 tilt_config.h 에 붙여넣으세요.\n");
            continue;
        }
        if (key == 'n') {
            set_ankle_zero(s_cur);
            print_zero_array();
            nvs_save_scratch();
            printf(">> 발목 2관절 영점 확정. NVS 에 임시 저장했습니다. 위 ZERO_TICK 배열을 tilt_config.h 에 붙여넣으세요.\n");
            continue;
        }

        for (const auto &km : JOG_KEYS) {
            if (km.key != key) continue;
            int j = km.joint;
            s_cur[j] = clamp_tick(static_cast<int32_t>(s_cur[j]) + km.dir * step_size());
            servo_write_tick(SERVO_ID[j], s_cur[j], 0);
            printf("%-14s tick=%4u  DH theta=%+7.2f deg  [%s]\n",
                   JOINT_LABEL[j], s_cur[j], tick_to_dh_deg(j, s_cur[j]), mode_name());
            break;
        }
    }
}

// ── 툴 5: 토크 해제 + 엔코더 모니터 ──────────────────────────────────
static void tool_monitor() {
    printf("\n── 엔코더 모니터 (전체 토크 OFF) ──────────────\n");
    printf("  손으로 자세를 잡으면 값이 따라옵니다.\n");
    printf("  z : 다리 6관절 영점 확정   n : 발목 영점 확정 (zero pose + 발바닥 평행)   x : 메뉴로\n\n");
    torque_all(false);

    uint16_t pos[NUM_JOINTS];
    while (true) {
        if (read_all(pos)) {
            printf("\r");
            for (int j = 0; j < NUM_JOINTS; ++j) {
                printf("%s:%4u(%+6.1f) ", JOINT_NAME[j], pos[j],
                       tick_to_dh_deg(j, pos[j]));
            }
            fflush(stdout);
        }

        int c = read_key_timeout(150);
        if (c < 0) continue;
        if (c == ' ') { emergency_stop(); continue; }

        char key = static_cast<char>(tolower(c));
        if (key == 'x') { printf("\n>> 메뉴로 (토크는 OFF 상태)\n"); return; }
        if (key == 'z' || key == 'n') {
            if (key == 'z') {
                set_leg_zero(pos);
            } else {
                printf("\n");
                set_ankle_zero(pos);
            }
            memcpy(s_cur, pos, sizeof(pos));
            print_zero_array();
            nvs_save_scratch();
            printf(">> 손으로 잡은 자세를 %s 영점으로 저장했습니다.\n",
                   key == 'z' ? "다리 6관절" : "발목 2관절");
            printf("   백래시·중력 때문에 오차가 있을 수 있으니, 조그(j)로 마무리하세요.\n\n");
        }
    }
}

// ── 툴 6: 관절 제한 지정 (관절별 수동) ───────────────────────────────
// 관절 하나씩: (−)쪽 끝까지 움직이고 Enter → y/n 확정 → (+)쪽 끝 → Enter → y/n
// → 그 관절은 0° 로 돌아가고 다음 관절로. 각도는 서보에서 읽은 실측값을 쓴다.
static constexpr float LIMIT_STEPS_DEG[] = {1.0f, 5.0f, 0.2f};

static int deg_to_ticks(float deg) {
    return static_cast<int>(lroundf(deg / DEG_PER_TICK));
}

// 한 관절만 현재 위치에서 토크 ON (free 모드 해제용)
static bool hold_joint(int j) {
    uint16_t t;
    if (!servo_read_tick(SERVO_ID[j], &t)) return false;
    s_cur[j] = t;
    servo_write_tick(SERVO_ID[j], t, 0);
    vTaskDelay(pdMS_TO_TICKS(3));
    return servo_torque(SERVO_ID[j], true);
}

// y / n 을 받는다. Enter(CR/LF) 잔여 입력은 무시. space = 비상정지(false, *estop = true).
static bool ask_yes_no(bool* estop) {
    *estop = false;
    while (true) {
        const int c = read_key_blocking();
        if (c == '\r' || c == '\n') continue;
        if (c == ' ') { *estop = true; return false; }
        const int k = tolower(c);
        if (k == 'y' || k == 'n') { printf("%c\n", k); return k == 'y'; }
    }
}

static void limit_print_help() {
    printf("\n  + / -  : 이 관절을 +/- 방향으로 한 스텝   m : 스텝 1° → 5° → 0.2°\n");
    printf("  f      : 이 관절만 토크 OFF/ON (손으로 움직이기)\n");
    printf("  Enter  : 지금 각도를 이 쪽 한계로 확정 (y/n)\n");
    printf("  >      : 이 쪽은 기존 값 유지하고 넘어가기   <  : 이전 관절로\n");
    printf("  ?      : 도움말   x : 종료하고 JOINT_LIMIT 출력   space : 비상정지\n");
}

static void limit_print_target(int j, int side) {
    printf("\n[%d/%d] %s — %s쪽 한계 (DH + = \"%s\")\n", j + 1, NUM_JOINTS, JOINT_LABEL[j],
           side == 0 ? "최소(−)" : "최대(+)", DH_PLUS_MEANING[j]);
    printf("  현재 설정: [%+.1f, %+.1f] deg%s\n", s_lim_deg[j][0], s_lim_deg[j][1],
           (s_lim_new[j][0] || s_lim_new[j][1]) ? "  (이번에 일부 확정됨)" : "");
}

// 관절 j 를 0° 로 천천히 되돌린다 (다음 관절로 넘어가기 전).
static void return_joint_to_zero(int j) {
    const uint16_t target = dh_rad_to_tick(j, 0.0f);
    servo_write_tick(SERVO_ID[j], target, 400);
    s_cur[j] = target;
    vTaskDelay(pdMS_TO_TICKS(700));
}

static void tool_limit() {
    printf("\n── 관절 제한 지정 (관절별 수동) ────────────────\n");
    printf("  관절을 하나씩 (−)끝 → (+)끝 순서로 움직여 Enter 로 확정합니다.\n");
    printf("  다른 관절은 토크로 현재 자세를 유지합니다. 공중에 매단 상태에서 하세요.\n");
    printf("  자기충돌·발판 간섭 직전까지만. 서보가 걸려 버티면 바로 -/+ 로 물러나세요.\n");
    limit_print_help();
    printf("\n  시작하려면 g, 취소는 아무 키나: ");
    fflush(stdout);
    int c = tolower(read_key_blocking());
    printf("%c\n", c);
    if (c != 'g') { printf(">> 취소\n"); return; }

    if (!hold_and_torque_on()) return;

    int j = 0;
    int side = 0;           // 0 = 최소(−), 1 = 최대(+)
    int step_idx = 0;
    bool free_mode = false;
    limit_print_target(j, side);

    while (j < NUM_JOINTS) {
        uint16_t t;
        if (servo_read_tick(SERVO_ID[j], &t)) {
            printf("\r  %s 현재 %+7.1f deg   step %.1f°%s        ", JOINT_NAME[j],
                   tick_to_dh_deg(j, t), LIMIT_STEPS_DEG[step_idx],
                   free_mode ? "  [FREE: 손으로]" : "");
            fflush(stdout);
        }

        c = read_key_timeout(150);
        if (c < 0) continue;
        if (c == ' ') { emergency_stop(); return; }

        if (c == '+' || c == '=' || c == '-' || c == '_') {
            if (free_mode) { printf("\n  FREE 모드입니다 — f 로 토크를 켠 뒤 +/- 를 쓰세요.\n"); continue; }
            const int dir = (c == '+' || c == '=') ? +1 : -1;
            // DH 방향 → raw tick 방향은 부호(JOINT_SIGN)를 따른다.
            const int dt = dir * s_sign[j] * deg_to_ticks(LIMIT_STEPS_DEG[step_idx]);
            s_cur[j] = clamp_tick(static_cast<int32_t>(s_cur[j]) + dt);
            servo_write_tick(SERVO_ID[j], s_cur[j], 0);
            continue;
        }

        switch (tolower(c)) {
            case 'm':
                step_idx = (step_idx + 1) % 3;
                break;
            case 'f':
                if (free_mode) {
                    free_mode = !hold_joint(j);
                    printf("\n  %s 토크 %s\n", JOINT_NAME[j], free_mode ? "ON 실패" : "ON (현재 위치 유지)");
                } else {
                    servo_torque(SERVO_ID[j], false);
                    free_mode = true;
                    printf("\n  %s 토크 OFF — 손으로 끝까지 움직인 뒤 Enter\n", JOINT_NAME[j]);
                }
                break;
            case '?':
                limit_print_help();
                limit_print_target(j, side);
                break;
            case 'x':
                if (free_mode) hold_joint(j);
                printf("\n>> 종료\n");
                print_limit_array();
                return;
            case '<':
                if (free_mode) { hold_joint(j); free_mode = false; }
                if (j > 0) { return_joint_to_zero(j); --j; }
                side = 0;
                limit_print_target(j, side);
                break;
            case '>':
            case '\r':
            case '\n': {
                bool advance = (c == '>');
                if (!advance) {
                    if (!servo_read_tick(SERVO_ID[j], &t)) {
                        printf("\n  위치 읽기 실패 — 다시 Enter\n");
                        break;
                    }
                    const float deg = tick_to_dh_deg(j, t);
                    printf("\n  %s %s 한계 = %+.1f deg 로 확정할까요? (y/n): ", JOINT_LABEL[j],
                           side == 0 ? "최소" : "최대", deg);
                    fflush(stdout);
                    bool estop = false;
                    const bool yes = ask_yes_no(&estop);
                    if (estop) { emergency_stop(); return; }
                    if (!yes) { printf("  취소 — 계속 움직이세요.\n"); break; }
                    if ((side == 0 && deg > 0.0f) || (side == 1 && deg < 0.0f)) {
                        printf("  ⚠ %s 한계가 0° 의 %s쪽에 있습니다 — 부호(s) 확인 권장.\n",
                               side == 0 ? "최소" : "최대", side == 0 ? "+" : "−");
                    }
                    s_lim_deg[j][side] = deg;
                    s_lim_new[j][side] = true;
                    advance = true;
                }
                if (!advance) break;
                if (side == 0) {
                    side = 1;
                } else {
                    if (s_lim_deg[j][0] >= s_lim_deg[j][1]) {
                        printf("  ⚠ 최소(%+.1f) ≥ 최대(%+.1f) — 이 관절을 다시 하려면 <, > 로 이동하세요.\n",
                               s_lim_deg[j][0], s_lim_deg[j][1]);
                    }
                    printf("  %s 확정: [%+.1f, %+.1f] deg — 0° 로 돌아갑니다.\n", JOINT_LABEL[j],
                           s_lim_deg[j][0], s_lim_deg[j][1]);
                    if (free_mode) { hold_joint(j); free_mode = false; }
                    return_joint_to_zero(j);
                    side = 0;
                    ++j;
                }
                if (j < NUM_JOINTS) limit_print_target(j, side);
                break;
            }
            default:
                break;
        }
    }

    printf("\n>> 8관절 모두 완료\n");
    print_limit_array();
}

// ── 툴 7: 회전 부호 판별 ─────────────────────────────────────────────
static void tool_sign() {
    constexpr int PROBE = 100;   // 약 8.8도

    printf("\n── 회전 부호 판별 ─────────────────────────────\n");
    printf("  관절을 raw + 방향으로 %d tick(%.1f deg) 움직입니다.\n",
           PROBE, PROBE * DEG_PER_TICK);
    printf("  그 움직임이 DH + 방향과 같은지 y/n 으로 답하세요.\n");
    printf("  ※ 공중에 매단 상태에서 진행하세요. x 로 중단.\n\n");

    if (!hold_and_torque_on()) return;

    for (int j = 0; j < NUM_JOINTS; ++j) {
        printf("[%d/%d] %-14s : DH + 는 \"%s\"\n",
               j + 1, NUM_JOINTS, JOINT_LABEL[j], DH_PLUS_MEANING[j]);
        printf("      움직입니다... ");
        fflush(stdout);

        uint16_t start = s_cur[j];
        uint16_t probe = clamp_tick(static_cast<int32_t>(start) + PROBE);
        servo_write_tick(SERVO_ID[j], probe, 600);
        vTaskDelay(pdMS_TO_TICKS(700));

        printf("DH + 방향이 맞습니까? (y/n, x=중단): ");
        fflush(stdout);

        int c = 0;
        while (true) {
            c = tolower(read_key_blocking());
            if (c == 'y' || c == 'n' || c == 'x') break;
            if (c == ' ') { emergency_stop(); return; }
        }
        printf("%c\n", c);

        // 원위치
        servo_write_tick(SERVO_ID[j], start, 600);
        vTaskDelay(pdMS_TO_TICKS(700));

        if (c == 'x') { printf(">> 중단. 지금까지 결과만 반영됩니다.\n"); break; }
        s_sign[j] = (c == 'y') ? +1 : -1;
    }

    print_sign_array();
    nvs_save_scratch();
}

// ── 툴 8: 영점(HOME) / zero pose 이동 — pose_test 방식 그대로 ────────
// pose_test 의 armServos() + PosePlayer + controlTask 와 같은 순서:
//   1) 8관절 위치를 모두 읽는다 (하나라도 실패하면 움직이지 않음)
//   2) 읽은 위치를 sync write 로 goal 에 기록한 뒤 토크 ON (튀지 않게)
//   3) rad 공간에서 smoothstep(3차) 보간, CONTROL_PERIOD_MS(10 ms)마다 sync write
// rad ↔ tick 변환은 pose_test JointMap.h 와 같이 tilt_config.h 의 ZERO_TICK / JOINT_SIGN 만 쓴다
// (작업값 s_zero 와 무관 — config 에 넣은 영점이 맞는지 확인하는 용도).
static constexpr uint32_t HOME_DURATION_MS = 2000;   // pose_test kHomeDurationMs
static constexpr uint32_t STAND_DURATION_MS = 2000;  // pose_test kStandDurationMs

static bool config_rad_to_tick(int j, float rad, uint16_t* tick_out) {
    const float raw = static_cast<float>(ZERO_TICK[j]) +
                      static_cast<float>(JOINT_SIGN[j]) * rad / RAD_PER_TICK;
    const long rounded = lroundf(raw);
    if (rounded < SERVO_POS_MIN || rounded > SERVO_POS_MAX) return false;
    *tick_out = static_cast<uint16_t>(rounded);
    return true;
}

static float config_tick_to_rad(int j, uint16_t tick) {
    return static_cast<float>(JOINT_SIGN[j]) *
           (static_cast<int>(tick) - static_cast<int>(ZERO_TICK[j])) * RAD_PER_TICK;
}

// pose_test armServos(): 전부 읽고, 그 위치를 goal 로 쓰고, 토크 ON.
static bool arm_like_pose_test(float measured_rad[NUM_JOINTS]) {
    uint16_t ticks[NUM_JOINTS];
    bool all_read = true;
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (!servo_read_tick(SERVO_ID[j], &ticks[j])) {
            printf("  arm 거부: %s (ID %u) 위치 읽기 실패\n", JOINT_NAME[j], SERVO_ID[j]);
            all_read = false;
        } else {
            measured_rad[j] = config_tick_to_rad(j, ticks[j]);
        }
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    if (!all_read) {
        printf("  8관절 위치가 모두 필요합니다 — 토크는 그대로 둡니다. 배선/전원/ID(p) 확인.\n");
        return false;
    }
    if (bus().syncWritePositions(SERVO_ID, ticks, NUM_JOINTS, 0, 0) != ESP_OK) {
        printf("  arm 거부: 현재 위치 유지 명령(sync write) 실패\n");
        return false;
    }
    if (bus().setTorqueAll(SERVO_ID, NUM_JOINTS, true) != ESP_OK) {
        printf("  토크 ON 실패 — 비상정지\n");
        emergency_stop();
        return false;
    }
    memcpy(s_cur, ticks, sizeof(ticks));
    return true;
}

static void move_pose_config(const float pose_rad[NUM_JOINTS], uint32_t duration_ms,
                             const char* name) {
    printf("\n>> %s 로 이동 (%lu ms, tilt_config.h 기준) — space = 비상정지\n", name,
           static_cast<unsigned long>(duration_ms));

    // 목표가 서보 범위 밖이면 출발하지 않는다 (pose_test convertCommand 와 같은 검사).
    uint16_t target_ticks[NUM_JOINTS];
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (!config_rad_to_tick(j, pose_rad[j], &target_ticks[j])) {
            printf("  거부: %s 목표가 서보 범위(0~4095) 밖입니다 — ZERO_TICK/JOINT_SIGN 확인\n",
                   JOINT_NAME[j]);
            return;
        }
    }

    float start[NUM_JOINTS];
    if (!arm_like_pose_test(start)) return;

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t elapsed = 0;
    uint16_t ticks[NUM_JOINTS];
    bool write_error = false;
    while (elapsed < duration_ms) {
        uint8_t ch = 0;
        while (usb_serial_jtag_read_bytes(&ch, 1, 0) > 0) {
            if (ch == ' ') { emergency_stop(); return; }
        }

        elapsed += CONTROL_PERIOD_MS;
        if (elapsed > duration_ms) elapsed = duration_ms;
        const float sn = static_cast<float>(elapsed) / static_cast<float>(duration_ms);
        const float smooth = sn * sn * (3.0f - 2.0f * sn);   // PosePlayer::step 과 동일
        for (int j = 0; j < NUM_JOINTS; ++j) {
            const float q = start[j] + (pose_rad[j] - start[j]) * smooth;
            if (!config_rad_to_tick(j, q, &ticks[j])) ticks[j] = s_cur[j];
        }
        if (bus().syncWritePositions(SERVO_ID, ticks, NUM_JOINTS, 0, 0) == ESP_OK) {
            memcpy(s_cur, ticks, sizeof(ticks));
        } else if (!write_error) {
            printf("  sync write 실패 — 마지막 명령 유지\n");
            write_error = true;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
    vTaskDelay(pdMS_TO_TICKS(300));  // 정착 대기

    printf("  reached %s\n", name);
    printf("\n  %-14s %8s %6s %6s %8s %8s\n", "joint", "target", "goal", "meas", "meas°", "err");
    printf("  ----------------------------------------------------------------\n");
    for (int j = 0; j < NUM_JOINTS; ++j) {
        const float want = pose_rad[j] * RAD2DEG;
        uint16_t m = 0;
        if (servo_read_tick(SERVO_ID[j], &m)) {
            const float got = config_tick_to_rad(j, m) * RAD2DEG;
            printf("  %-14s %+7.1f° %6u %6u %+7.1f° %+7.1f°%s\n", JOINT_LABEL[j], want,
                   target_ticks[j], m, got, got - want, fabsf(got - want) > 2.0f ? "  ← 큼" : "");
        } else {
            printf("  %-14s %+7.1f° %6u %6s\n", JOINT_LABEL[j], want, target_ticks[j], "?");
        }
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    printf("  (goal = config ZERO_TICK 기준 목표 tick, err = 실측 − 목표. 1° 안팎은 정상)\n\n");
}

static void tool_goto_home() {
    static const float home[NUM_JOINTS] = {};   // pose_test homePose(): 전 관절 0 rad
    move_pose_config(home, HOME_DURATION_MS, "HOME [0, 0, 0, 0]");
    printf(">> 다리가 곧게 펴졌는지 확인: roll = 수직, 허벅지 수직, 정강이 일직선.\n");
    printf("   발목 0° = 발바닥이 정강이와 직각 → 다리를 편 HOME 에서 발바닥이 지면과 평행해야 정상.\n\n");
}

static void tool_goto_zero_pose() {
    move_pose_config(ZERO_POSE_RAD, STAND_DURATION_MS, "STAND [0, -20, +40, -20]");
    printf(">> 발판이 지면과 평행하고 발바닥 점이 hip pitch 축 바로 아래인지 확인하세요.\n");
    printf("   (지면~hip pitch 축 높이가 약 %.0fmm 여야 함)\n\n", ZERO_POSE_HEIGHT_MM);
}

// ── 툴 9: 발목 영점 (stand 자세에서 발바닥을 바닥에 맞춰 잡기) ──────
// 발목 영점 = zero pose 에서 발바닥이 바닥과 평행한 각도(tilt_config.h 참고).
// hip/knee 를 zero pose 로 세우고 발목만 토크를 풀어, 평평한 바닥이 발바닥을
// 눌러 맞추게 한 뒤 그 각도를 읽는다.
static void hold_ankles(const uint16_t pos[NUM_JOINTS]) {
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (!is_ankle(j)) continue;
        s_cur[j] = pos[j];
        servo_write_tick(SERVO_ID[j], pos[j], 0);
        vTaskDelay(pdMS_TO_TICKS(3));
        servo_torque(SERVO_ID[j], true);
        vTaskDelay(pdMS_TO_TICKS(3));
    }
}

static void tool_ankle_zero() {
    printf("\n── 발목 영점 ─────────────────────────────────\n");
    printf("  먼저 다리 6관절 영점(o/j → z)과 부호(s)가 끝나 있어야 합니다.\n");
    printf("  1) hip/knee 를 zero pose [0, -20, +40] 로 천천히 이동\n");
    printf("  2) 발목 두 개만 토크 OFF\n");
    printf("  3) 몸통을 잡고 평평한 바닥에 내려 두 발바닥이 완전히 붙게 한 뒤 z\n");
    printf("  계속하려면 g, 취소는 아무 키나: ");
    fflush(stdout);

    int c = tolower(read_key_blocking());
    printf("%c\n", c);
    if (c != 'g') { printf(">> 취소\n"); return; }

    if (!hold_and_torque_on()) return;

    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (is_ankle(j)) continue;
        uint16_t target = dh_rad_to_tick(j, ZERO_POSE_RAD[j]);
        printf("  %-14s %4u -> %4u\n", JOINT_LABEL[j], s_cur[j], target);
        s_cur[j] = target;
        servo_write_tick(SERVO_ID[j], target, 300);   // 느리게
        vTaskDelay(pdMS_TO_TICKS(400));
    }
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (!is_ankle(j)) continue;
        servo_torque(SERVO_ID[j], false);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    printf("\n  발목 토크 OFF. 발바닥을 바닥에 붙이세요.\n");
    printf("  z : 지금 발목 각을 영점으로 확정   x : 확정 없이 종료 (발목은 현재 위치에서 토크 ON)\n\n");

    uint16_t pos[NUM_JOINTS];
    while (true) {
        if (read_all(pos)) {
            printf("\r");
            for (int j = 0; j < NUM_JOINTS; ++j) {
                if (!is_ankle(j)) continue;
                printf("%s: tick %4u  DH %+6.1f deg (현재 영점 기준)   ", JOINT_NAME[j], pos[j],
                       tick_to_dh_deg(j, pos[j]));
            }
            fflush(stdout);
        }

        c = read_key_timeout(150);
        if (c < 0) continue;
        if (c == ' ') { emergency_stop(); return; }

        const char key = static_cast<char>(tolower(c));
        if (key == 'x') {
            if (read_all(pos)) hold_ankles(pos);
            printf("\n>> 메뉴로 (영점 변경 없음)\n");
            return;
        }
        if (key == 'z') {
            if (!read_all(pos)) {
                printf("\n  위치 읽기 실패 — 다시 z 를 누르세요.\n");
                continue;
            }
            printf("\n");
            set_ankle_zero(pos);
            hold_ankles(pos);
            print_zero_array();
            nvs_save_scratch();
            printf(">> 발목 영점 확정, 현재 위치에서 토크 ON.\n");
            printf("   미세 조정: j (u/j = 왼발목, o/l = 오른발목) 로 맞춘 뒤 n.\n\n");
            return;
        }
    }
}

// ── 메뉴 ─────────────────────────────────────────────────────────────
static void print_menu() {
    printf("\n");
    printf("╔══════════════════════════════════════════════╗\n");
    printf("║  TILT 서보 셋업 / 캘리브레이션 툴 (8관절)    ║\n");
    printf("╚══════════════════════════════════════════════╝\n");
    printf("  p : ID 스캔 (0~253)\n");
    printf("  i : 서보 ID 변경 (한 번에 한 개만 연결)\n");
    printf("  b : 보레이트 스캔\n");
    printf("  j : 정밀 조그  ← 영점 잡기 본체 (z = 다리 6관절, n = 발목)\n");
    printf("  o : 토크 해제 + 엔코더 모니터\n");
    printf("  k : 발목 영점 (zero pose 로 세우고 발바닥을 바닥에 맞춤)\n");
    printf("  l : 관절 제한 지정 (관절별 수동, Enter + y/n) -> JOINT_LIMIT\n");
    printf("  s : 회전 부호 판별 -> JOINT_SIGN\n");
    printf("  h : HOME — 전 관절 0° 로 이동 (pose_test h 와 동일, tilt_config.h 영점 검증)\n");
    printf("  u : STAND — zero pose [0, -20, +40, -20] 으로 이동 (tilt_config.h 기준)\n");
    printf("  v : 현재 작업값 배열 전부 출력 (영점·부호·제한)\n");
    printf("  r : NVS 스크래치를 작업값으로 불러오기\n");
    printf("  c : NVS 스크래치 비우기\n");
    printf("  t : 전체 토크 ON/OFF 토글\n");
    printf("  ? : 이 메뉴\n");
    printf("  space : 비상정지\n\n");
}

static void menu_task(void *) {
    print_menu();
    bool torque_on = s_boot_torque_on;

    while (true) {
        int c = read_key_blocking();
        if (c == ' ') { emergency_stop(); torque_on = false; continue; }

        switch (tolower(c)) {
            case 'p': tool_scan_ids();        break;
            case 'i':
                if (tool_change_id()) torque_on = false;
                break;
            case 'b': tool_scan_baud();       break;
            case 'j': tool_jog();             break;
            case 'o': tool_monitor();         break;
            case 'k': tool_ankle_zero();      break;
            case 'l': tool_limit();           break;
            case 's': tool_sign();            break;
            case 'h':
            case '0': tool_goto_home();       break;
            case 'u': tool_goto_zero_pose();  break;
            case 'v':
                print_zero_array();
                print_sign_array();
                print_limit_array();
                break;
            case 'r': nvs_load_scratch(true); break;
            case 'c': nvs_clear_scratch();    break;
            case 't':
                if (torque_on) {
                    torque_all(false);
                    torque_on = false;
                } else {
                    torque_on = hold_and_torque_on();
                }
                printf(">> 전체 토크 %s\n", torque_on ? "ON" : "OFF");
                break;
            case '?': print_menu();           break;
            default: break;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────
extern "C" void app_main(void) {
    // config 값으로 초기화
    for (int j = 0; j < NUM_JOINTS; ++j) {
        s_zero[j] = ZERO_TICK[j];
        s_sign[j] = JOINT_SIGN[j];
    }

    init_limits_from_config();

    nvs_init_once();
    nvs_load_scratch(false);

    usb_serial_jtag_driver_config_t ucfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    esp_err_t err = usb_serial_jtag_driver_install(&ucfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);

    if (!servo_bus_init()) {
        printf("서보 버스 초기화 실패\n");
    }

    // pose_test 와 같이 가속도 레지스터를 0 으로 (다른 테스트가 남긴 가속도 설정 제거)
    for (int j = 0; j < NUM_JOINTS; ++j) {
        if (bus().setAcceleration(SERVO_ID[j], 0) != ESP_OK) {
            printf("  %-14s ID %u: acceleration=0 실패 (응답 없음?)\n", JOINT_LABEL[j], SERVO_ID[j]);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // 부팅 시 절대 위치 명령은 보내지 않는다 — 현재 위치를 읽어 기준으로만 삼음
    printf("\n부팅: 현재 위치 읽는 중...\n");
    for (int j = 0; j < NUM_JOINTS; ++j) {
        uint16_t t = 2048;
        if (servo_read_tick(SERVO_ID[j], &t)) {
            printf("  %-14s tick=%4u  DH theta=%+7.2f deg\n",
                   JOINT_LABEL[j], t, tick_to_dh_deg(j, t));
        } else {
            printf("  %-14s READ 실패 — 2048 가정\n", JOINT_LABEL[j]);
        }
        s_cur[j] = t;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    s_boot_torque_on = hold_and_torque_on();
    if (s_boot_torque_on) {
        printf("토크 ON 완료 (현재 위치 유지).\n");
    } else {
        printf("토크 OFF 상태로 시작합니다. 문제 해결 후 t 로 켜세요.\n");
    }

    xTaskCreate(menu_task, "menu", 6144, nullptr, 5, nullptr);
}
