// TILT — 서보 셋업 / 캘리브레이션 툴
//
// 목적: tilt_config.h 의 TODO 항목(ZERO_TICK, JOINT_SIGN, JOINT_LIMIT)을
//       실측으로 채우는 것. 모든 결과는 그대로 붙여넣을 수 있는 C 코드로 출력한다.
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

static const char *NVS_NAMESPACE = "tilt_cal";
static const char *NVS_KEY_ZERO  = "zero_tick";
static const char *NVS_KEY_SIGN  = "joint_sign";

// ── 작업용 상태 ──────────────────────────────────────────────────────
// config 값으로 초기화한 뒤, NVS 스크래치가 있으면 덮어쓴다.
static uint16_t s_zero[NUM_JOINTS];
static int8_t   s_sign[NUM_JOINTS];

// 조그 중 추적하는 목표 위치(절대 점프 방지용)
static uint16_t s_cur[NUM_JOINTS];

// 가동범위 탐색 결과
static uint16_t s_range_min[NUM_JOINTS];
static uint16_t s_range_max[NUM_JOINTS];

static bool s_coarse = false;   // false = FINE(1 tick), true = COARSE
static bool s_boot_torque_on = false;  // 부팅 시 토크 ON 성공 여부 (메뉴 t 토글 초기값)

static constexpr int FINE_STEP   = 1;
static constexpr int COARSE_STEP = 20;

static const char *JOINT_LABEL[NUM_JOINTS] = {
    "L_HIP_ROLL", "L_HIP_PITCH", "L_KNEE_PITCH",
    "R_HIP_ROLL", "R_HIP_PITCH", "R_KNEE_PITCH",
};

static const char *DH_PLUS_MEANING[NUM_JOINTS] = {
    "발이 왼쪽(바깥)으로", "다리가 뒤로 스윙", "무릎이 굽음",
    "발이 왼쪽(안쪽)으로", "다리가 뒤로 스윙", "무릎이 굽음",
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

// ── NVS 스크래치 ─────────────────────────────────────────────────────
static void nvs_init_once() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
}

static void nvs_load_scratch() {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;

    size_t sz = sizeof(s_zero);
    if (nvs_get_blob(h, NVS_KEY_ZERO, s_zero, &sz) == ESP_OK && sz == sizeof(s_zero)) {
        printf("[NVS] 임시 저장된 ZERO_TICK 을 불러왔습니다 (config 값이 아님).\n");
    }
    sz = sizeof(s_sign);
    if (nvs_get_blob(h, NVS_KEY_SIGN, s_sign, &sz) == ESP_OK && sz == sizeof(s_sign)) {
        printf("[NVS] 임시 저장된 JOINT_SIGN 을 불러왔습니다 (config 값이 아님).\n");
    }
    nvs_close(h);
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
    printf(">> NVS 스크래치를 비웠습니다. 다음 부팅부터 config 값을 씁니다.\n");
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

static void print_limit_array() {
    printf("\n// ── tilt_config.h 의 JOINT_LIMIT 을 아래로 교체 ──\n");
    printf("//    (실측 가동범위. 실제로는 여기서 여유를 두고 좁히는 걸 권장)\n");
    printf("inline constexpr JointLimit JOINT_LIMIT[2][3] = {\n");
    for (int leg = 0; leg < 2; ++leg) {
        printf("    {\n");
        for (int leg_joint = 0; leg_joint < 3; ++leg_joint) {
            const int j = leg * 3 + leg_joint;
            const float a = tick_to_dh_deg(j, s_range_min[j]);
            const float b = tick_to_dh_deg(j, s_range_max[j]);
            const float lo = (a < b) ? a : b;
            const float hi = (a < b) ? b : a;
            printf("        { %+7.1ff * DEG2RAD, %+7.1ff * DEG2RAD },  // %s\n",
                   lo, hi, JOINT_LABEL[j]);
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
    }
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
static const KeyMap JOG_KEYS[12] = {
    {'q', 0, +1}, {'a', 0, -1},
    {'w', 1, +1}, {'s', 1, -1},
    {'e', 2, +1}, {'d', 2, -1},
    {'r', 3, +1}, {'f', 3, -1},
    {'t', 4, +1}, {'g', 4, -1},
    {'y', 5, +1}, {'h', 5, -1},
};

static void jog_print_help() {
    printf("\n── 정밀 조그 ──────────────────────────────────\n");
    printf("  q/a  w/s  e/d : 왼쪽  ROLL / PITCH / KNEE  (+/-)\n");
    printf("  r/f  t/g  y/h : 오른쪽 ROLL / PITCH / KNEE  (+/-)\n");
    printf("  m : FINE(%d tick) <-> COARSE(%d tick) 토글\n", FINE_STEP, COARSE_STEP);
    printf("  i : 6축 현재 상태 표 출력\n");
    printf("  z : 지금 자세를 영점으로 확정 (배열 출력 + NVS 임시저장)\n");
    printf("  x : 메뉴로\n");
    printf("  현재 모드: %s (%.3f deg/step)\n\n",
           mode_name(), step_size() * DEG_PER_TICK);
    printf("  ※ DH theta 가 0.00 에 수렴하도록 맞추세요.\n");
    printf("    ROLL=다리 수직(좌우 안 기울게) / PITCH=허벅지 수직 / KNEE=정강이 일직선\n\n");
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
            for (int j = 0; j < NUM_JOINTS; ++j) s_zero[j] = s_cur[j];
            print_zero_array();
            nvs_save_scratch();
            printf(">> NVS 에 임시 저장했습니다. 위 배열을 tilt_config.h 에 붙여넣으세요.\n");
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
    printf("  z : 현재 자세를 영점으로 확정   x : 메뉴로\n\n");
    torque_all(false);

    uint16_t pos[NUM_JOINTS];
    while (true) {
        if (read_all(pos)) {
            printf("\r");
            for (int j = 0; j < NUM_JOINTS; ++j) {
                printf("%s:%4u(%+6.1f) ", tilt::JOINT_NAME[j], pos[j],
                       tick_to_dh_deg(j, pos[j]));
            }
            fflush(stdout);
        }

        int c = read_key_timeout(150);
        if (c < 0) continue;
        if (c == ' ') { emergency_stop(); continue; }

        char key = static_cast<char>(tolower(c));
        if (key == 'x') { printf("\n>> 메뉴로 (토크는 OFF 상태)\n"); return; }
        if (key == 'z') {
            for (int j = 0; j < NUM_JOINTS; ++j) s_zero[j] = pos[j];
            memcpy(s_cur, pos, sizeof(pos));
            print_zero_array();
            nvs_save_scratch();
            printf(">> 손으로 잡은 자세를 영점으로 저장했습니다.\n");
            printf("   백래시·중력 때문에 오차가 있을 수 있으니, 조그(j)로 마무리하세요.\n\n");
        }
    }
}

// ── 툴 6: 가동범위 탐색 ──────────────────────────────────────────────
static void tool_range() {
    printf("\n── 가동범위 탐색 (전체 토크 OFF) ──────────────\n");
    printf("  각 관절을 손으로 끝에서 끝까지 천천히 움직이세요.\n");
    printf("  자기충돌(다리끼리 부딪힘)이 나기 직전까지만!\n");
    printf("  x : 종료하고 JOINT_LIMIT 배열 출력\n\n");
    torque_all(false);

    uint16_t pos[NUM_JOINTS];
    if (!read_all(pos)) {
        printf("  위치 읽기 실패 — 메뉴로 돌아갑니다.\n");
        return;
    }
    for (int j = 0; j < NUM_JOINTS; ++j) {
        s_range_min[j] = s_range_max[j] = pos[j];
    }

    while (true) {
        if (read_all(pos)) {
            for (int j = 0; j < NUM_JOINTS; ++j) {
                if (pos[j] < s_range_min[j]) s_range_min[j] = pos[j];
                if (pos[j] > s_range_max[j]) s_range_max[j] = pos[j];
            }
            printf("\r");
            for (int j = 0; j < NUM_JOINTS; ++j) {
                printf("%s:[%+6.1f,%+6.1f] ", tilt::JOINT_NAME[j],
                       tick_to_dh_deg(j, s_range_min[j]),
                       tick_to_dh_deg(j, s_range_max[j]));
            }
            fflush(stdout);
        }

        int c = read_key_timeout(150);
        if (c < 0) continue;
        if (c == ' ') { emergency_stop(); continue; }
        if (tolower(c) == 'x') {
            printf("\n");
            print_limit_array();
            return;
        }
    }
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

// ── 툴 8: zero pose 이동 (config 검증) ───────────────────────────────
static void tool_goto_zero_pose() {
    printf("\n── zero pose 이동 ─────────────────────────────\n");
    printf("  config 의 ZERO_POSE_RAD 로 천천히 이동합니다.\n");
    printf("  [0, -20, +40] deg — 발바닥이 지면과 평행한 기본자세\n");
    printf("  ※ 로봇이 넘어지지 않게 잡거나 매단 상태에서!\n");
    printf("  계속하려면 g, 취소는 아무 키나: ");
    fflush(stdout);

    int c = tolower(read_key_blocking());
    printf("%c\n", c);
    if (c != 'g') { printf(">> 취소\n"); return; }

    if (!hold_and_torque_on()) return;

    for (int j = 0; j < NUM_JOINTS; ++j) {
        uint16_t target = dh_rad_to_tick(j, ZERO_POSE_RAD[j]);
        printf("  %-14s %4u -> %4u\n", JOINT_LABEL[j], s_cur[j], target);
        s_cur[j] = target;
        servo_write_tick(SERVO_ID[j], target, 300);   // 느리게
        vTaskDelay(pdMS_TO_TICKS(400));
    }
    printf(">> 완료. 발판이 지면과 평행하고 발바닥 점이 hip pitch 축 바로 아래인지 확인하세요.\n");
    printf("   (지면~hip pitch 축 높이가 약 %.0fmm 여야 함)\n\n", ZERO_POSE_HEIGHT_MM);
}

// ── 메뉴 ─────────────────────────────────────────────────────────────
static void print_menu() {
    printf("\n");
    printf("╔══════════════════════════════════════════════╗\n");
    printf("║  TILT 서보 셋업 / 캘리브레이션 툴            ║\n");
    printf("╚══════════════════════════════════════════════╝\n");
    printf("  p : ID 스캔 (0~253)\n");
    printf("  i : 서보 ID 변경 (한 번에 한 개만 연결)\n");
    printf("  b : 보레이트 스캔\n");
    printf("  j : 정밀 조그  ← 영점 잡기 본체\n");
    printf("  o : 토크 해제 + 엔코더 모니터\n");
    printf("  l : 가동범위 탐색 -> JOINT_LIMIT\n");
    printf("  s : 회전 부호 판별 -> JOINT_SIGN\n");
    printf("  0 : zero pose 로 이동 (config 검증)\n");
    printf("  v : 현재 작업값 배열 전부 출력\n");
    printf("  c : NVS 스크래치 비우기\n");
    printf("  t : 전체 토크 ON/OFF 토글\n");
    printf("  h : 이 메뉴\n");
    printf("  space : 비상정지\n\n");
}

static void menu_task(void *arg) {
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
            case 'l': tool_range();           break;
            case 's': tool_sign();            break;
            case '0': tool_goto_zero_pose();  break;
            case 'v':
                print_zero_array();
                print_sign_array();
                break;
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
            case 'h': print_menu();           break;
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

    nvs_init_once();
    nvs_load_scratch();

    usb_serial_jtag_driver_config_t ucfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    esp_err_t err = usb_serial_jtag_driver_install(&ucfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);

    if (!servo_bus_init()) {
        printf("서보 버스 초기화 실패\n");
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
