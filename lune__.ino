#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "driver/i2s.h"
#include "esp_heap_caps.h"
#include <math.h>

/* ================= I2S ================= */
#define I2S_PORT I2S_NUM_0
#define PIN_BCK    5
#define PIN_WS     6
#define PIN_DOUT   4
#define PIN_DIN    7
#define PIN_MCLK   0

/* ================= I2C / ADS1115 ================= */
#define I2C_SDA 9
#define I2C_SCL 10
Adafruit_ADS1115 ads;

/* ================= ADC ================= */
#define ADC_SPEED        1
#define ADC_FEEDBACK     2
#define ADC_REVERSE_MIX  3

/* ================= Audio ================= */
#define SAMPLE_RATE 44100
#define DMA_LEN     256

/* ================= Delay ================= */
#define MAX_DELAY_MS 5000
#define MIN_DELAY_MS 20
#define MAX_DELAY_SAMPLES (SAMPLE_RATE * MAX_DELAY_MS / 1000)
#define MIN_DELAY_SAMPLES (SAMPLE_RATE * MIN_DELAY_MS / 1000)

/* ================= Reverse ================= */
#define REV_XFADE_MS 30
#define REV_XFADE_SAMPLES (SAMPLE_RATE * REV_XFADE_MS / 1000)
#define PIN_FREEZE_BUTTON 11

/* ================= Speed Control ================= */
#define MAX_SPEED 4.0f           // 最大4倍速
#define SPEED_DEADZONE 0.05f     // 中央死区

/* ================= Buffers ================= */
static float *delayL, *delayR;
static float *revA_L, *revA_R;
static float *revB_L, *revB_R;
static int32_t i2s_buf[DMA_LEN * 2];

/* ================= Utility Functions ================= */
inline float clamp_energy(float x) {
    const float hard_limit = 0.95f;
    const float soft_threshold = 0.85f;
    
    if (x > hard_limit) {
        return hard_limit + (x - hard_limit) * 0.2f;
    }
    if (x < -hard_limit) {
        return -hard_limit + (x + hard_limit) * 0.2f;
    }
    
    if (x > soft_threshold) {
        return soft_threshold + (x - soft_threshold) * 0.7f;
    }
    if (x < -soft_threshold) {
        return -soft_threshold + (x + soft_threshold) * 0.7f;
    }
    
    return x;
}

inline float smoothFade(float x) {
    return x * x * (3.0f - 2.0f * x);
}

/* ================= 相位检测辅助函数 ================= */
float calculateCurrentPhase(float current_delay, int delay_write_pos, float* delayL_buf, float* delayR_buf) {
    static float last_peak_pos = 0.0f;
    
    int search_window = min(128, (int)current_delay);
    float max_energy = 0.0f;
    int peak_idx = 0;
    
    for (int offset = 0; offset < search_window; offset++) {
        int idx = (delay_write_pos - offset + MAX_DELAY_SAMPLES) % MAX_DELAY_SAMPLES;
        float energy = fabs(delayL_buf[idx]) + fabs(delayR_buf[idx]);
        if (energy > max_energy) {
            max_energy = energy;
            peak_idx = idx;
        }
    }
    
    if (max_energy < 0.001f) {
        return last_peak_pos;
    }
    
    float distance = (delay_write_pos - peak_idx + MAX_DELAY_SAMPLES) % MAX_DELAY_SAMPLES;
    float phase = distance / current_delay;
    
    if (fabs(phase - last_peak_pos) < 0.1f || last_peak_pos == 0.0f) {
        last_peak_pos = phase;
    }
    
    return last_peak_pos;
}

/* ================= Freeze State ================= */
struct FreezeState {
    bool active = false;
    float base_delay = 0.0f;
    float feedback = 0.0f;
    float reverse_mix = 0.0f;
    bool reverse_active = false;
    int rev_len = 0;
    
    // 速度控制 - 修改为带符号的速度值
    float speed_target = 0.0f;     // 目标速度（带符号，负值表示倒放）
    float speed_current = 0.0f;     // 当前速度（带符号）
    float read_pointer = 0.0f;      // 读取指针
    float rev_read_pointer = 0.0f;  // 倒放读取指针
    
    float freeze_phase = 0.0f;
    float freeze_instant_speed = 1.0f;
    
    float detectFreezeSpeed(float current_delay, int delay_write_pos) {
        static float last_avg_energy = 0.0f;
        static uint32_t last_time = 0;
        
        float current_avg_energy = 0.0f;
        for (int i = 0; i < 16; i++) {
            int idx = (delay_write_pos - i + MAX_DELAY_SAMPLES) % MAX_DELAY_SAMPLES;
            current_avg_energy += fabs(delayL[idx]) + fabs(delayR[idx]);
        }
        current_avg_energy /= 16.0f;
        
        uint32_t now = millis();
        float detected_speed = 1.0f;
        
        if (last_time > 0 && now > last_time) {
            float energy_diff = current_avg_energy - last_avg_energy;
            float time_diff = (now - last_time) / 1000.0f;
            
            if (time_diff > 0.01f) {
                float energy_rate = fabs(energy_diff) / time_diff;
                
                if (energy_rate < 0.01f) detected_speed = 0.5f;
                else if (energy_rate < 0.05f) detected_speed = 1.0f;
                else if (energy_rate < 0.1f) detected_speed = 1.5f;
                else detected_speed = 2.0f;
                
                detected_speed = constrain(detected_speed, 0.1f, 4.0f);
            }
        }
        
        last_avg_energy = current_avg_energy;
        last_time = now;
        
        return detected_speed;
    }
    
    void init(float delay, float fb, float mix, bool rev, int rlen, int write_pos) {
        base_delay = delay;
        feedback = fb;
        reverse_mix = mix;
        reverse_active = rev;
        rev_len = rlen;
        
        freeze_instant_speed = detectFreezeSpeed(delay, write_pos);
        freeze_phase = calculateCurrentPhase(delay, write_pos, delayL, delayR);
        
        read_pointer = write_pos - (freeze_phase * base_delay);
        if (read_pointer < 0) read_pointer += MAX_DELAY_SAMPLES;
        if (read_pointer >= MAX_DELAY_SAMPLES) read_pointer -= MAX_DELAY_SAMPLES;
        
        // 初始化速度为0（中央位置）
        speed_target = 0.0f;
        speed_current = 0.0f;
        rev_read_pointer = 0.0f;
    }
    
    // ===== 修改后的速度更新函数 =====
    void updateSpeed(float knob) {
        // knob范围 0-1，转换为 -1 到 1
        float centered = knob * 2.0f - 1.0f;
        
        // 死区处理
        if (fabs(centered) < SPEED_DEADZONE) {
            speed_target = 0.0f;
        } else {
            // 重新映射，去除死区影响
            float mapped;
            if (centered > 0) {
                // 右侧：正放，速度范围 0 到 +MAX_SPEED
                mapped = (centered - SPEED_DEADZONE) / (1.0f - SPEED_DEADZONE);
                speed_target = mapped * MAX_SPEED;
            } else {
                // 左侧：倒放，速度范围 -MAX_SPEED 到 0
                mapped = (fabs(centered) - SPEED_DEADZONE) / (1.0f - SPEED_DEADZONE);
                speed_target = -mapped * MAX_SPEED;  // 负值表示倒放
            }
        }
        
        // 平滑速度变化
        speed_current += 0.05f * (speed_target - speed_current);
        
        // 确保速度在合理范围内
        if (fabs(speed_current) < 0.01f) speed_current = 0.0f;
    }
    
    // 获取当前速度的绝对值（用于显示）
    float getCurrentSpeed() {
        return fabs(speed_current);
    }
    
    // 判断是否倒放
    bool isReversing() {
        return speed_current < 0;
    }
};

static FreezeState freeze;

/* ================= State ================= */
static int delay_write = 0;
static float delay_samples_f = MIN_DELAY_SAMPLES;
static float delay_target = MIN_DELAY_SAMPLES;
static float feedback = 0.35f;
static float feedback_target = 0.35f;

/* ================= Reverse State ================= */
static int rev_len = MIN_DELAY_SAMPLES;
static int rev_write_pos = 0;
static int rev_read_pos = 0;
static bool rev_using_A = true;
static bool reverse_active = false;

/* ================= Crossfade ================= */
static bool rev_fading = false;
static int rev_fade_pos = 0;
static float rev_fade_in_gain = 1.0f;
static float rev_fade_out_gain = 0.0f;

/* ================= Mix ================= */
static float reverse_mix = 0.0f;
static float reverse_mix_target = 0.0f;

/* ================= Fade Functions ================= */
void updateFadeGains() {
    if (!rev_fading) {
        rev_fade_in_gain = 1.0f;
        rev_fade_out_gain = 0.0f;
        return;
    }
    
    float t = (float)rev_fade_pos / REV_XFADE_SAMPLES;
    float fade = smoothFade(t);
    
    rev_fade_in_gain = fade;
    rev_fade_out_gain = 1.0f - fade;
    
    rev_fade_pos++;
    if (rev_fade_pos >= REV_XFADE_SAMPLES) {
        rev_fading = false;
        rev_fade_in_gain = 1.0f;
        rev_fade_out_gain = 0.0f;
    }
}

/* ================= Setup ================= */
void setup() {
    Serial.begin(115200);
    Serial.println("=== Reverse Delay with Freeze (Speed Control) ===");
    
    pinMode(PIN_FREEZE_BUTTON, INPUT_PULLUP);
    analogReadResolution(12);
    
    pinMode(ADC_SPEED, INPUT);
    pinMode(ADC_FEEDBACK, INPUT);
    pinMode(ADC_REVERSE_MIX, INPUT);
    
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!ads.begin()) {
        Serial.println("Failed to initialize ADS1115!");
        while(1);
    }
    ads.setGain(GAIN_ONE);
    
    size_t delay_bytes = MAX_DELAY_SAMPLES * sizeof(float);
    Serial.printf("Allocating %d KB total\n", (delay_bytes * 6) / 1024);
    
    delayL = (float*)heap_caps_malloc(delay_bytes, MALLOC_CAP_SPIRAM);
    delayR = (float*)heap_caps_malloc(delay_bytes, MALLOC_CAP_SPIRAM);
    revA_L = (float*)heap_caps_malloc(delay_bytes, MALLOC_CAP_SPIRAM);
    revA_R = (float*)heap_caps_malloc(delay_bytes, MALLOC_CAP_SPIRAM);
    revB_L = (float*)heap_caps_malloc(delay_bytes, MALLOC_CAP_SPIRAM);
    revB_R = (float*)heap_caps_malloc(delay_bytes, MALLOC_CAP_SPIRAM);
    
    if (!delayL || !delayR || !revA_L || !revA_R || !revB_L || !revB_R) {
        Serial.println("Memory allocation failed!");
        while(1);
    }
    
    for (int i = 0; i < MAX_DELAY_SAMPLES; i++) {
        delayL[i] = 0;
        delayR[i] = 0;
        revA_L[i] = 0;
        revA_R[i] = 0;
        revB_L[i] = 0;
        revB_R[i] = 0;
    }
    
    i2s_config_t cfg = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .dma_buf_count = 8,
        .dma_buf_len = DMA_LEN,
        .use_apll = true,
        .tx_desc_auto_clear = true,
        .fixed_mclk = SAMPLE_RATE * 256
    };
    
    i2s_pin_config_t pins = {
        .mck_io_num = PIN_MCLK,
        .bck_io_num = PIN_BCK,
        .ws_io_num = PIN_WS,
        .data_out_num = PIN_DOUT,
        .data_in_num = PIN_DIN
    };
    
    i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
    i2s_set_pin(I2S_PORT, &pins);
    
    Serial.println("System ready!");
    Serial.println("Freeze Mode Speed Control:");
    Serial.println("  Center: Stop");
    Serial.println("  Left: Reverse playback (up to 4x)");
    Serial.println("  Right: Forward playback (up to 4x)");
}

/* ================= Control Processing ================= */
void processControls() {
    static bool last_btn = HIGH;
    static uint32_t last_ctrl_time = 0;
    
    if (millis() - last_ctrl_time < 15) return;
    
    bool btn = digitalRead(PIN_FREEZE_BUTTON);
    
    if (btn == LOW && last_btn == HIGH) {
        freeze.active = !freeze.active;
        
        if (freeze.active) {
            freeze.init(delay_samples_f, feedback, reverse_mix, 
                       reverse_active, rev_len, delay_write);
            Serial.println("Freeze ON");
        } else {
            Serial.println("Freeze OFF");
        }
    }
    last_btn = btn;
    
    if (!freeze.active) {
        int16_t raw = ads.readADC_SingleEnded(0);
        raw = constrain(raw, 0, 26000);
        
        delay_target = MIN_DELAY_SAMPLES + (float)raw * (MAX_DELAY_SAMPLES - MIN_DELAY_SAMPLES) / 26000.0f;
        feedback_target = analogRead(ADC_FEEDBACK) / 4095.0f;
        reverse_mix_target = analogRead(ADC_REVERSE_MIX) / 4095.0f;
    } else {
        float speed_knob = analogRead(ADC_SPEED) / 4095.0f;
        freeze.updateSpeed(speed_knob);
        
        // 调试输出，每100次打印一次
        static int debug_counter = 0;
        if (++debug_counter >= 100) {
            debug_counter = 0;
            float centered = speed_knob * 2.0f - 1.0f;
            Serial.printf("Speed knob: %.3f | Center: %.3f | Speed: %.3f %s\n", 
                         speed_knob, centered, 
                         freeze.getCurrentSpeed(),
                         freeze.isReversing() ? "(REV)" : "(FWD)");
        }
    }
    
    last_ctrl_time = millis();
}

/* ================= Audio Processing - Normal Mode ================= */
void processNormalMode(float inL, float inR, float& outL, float& outR) {
    // 平滑参数变化
    delay_samples_f += 0.0005f * (delay_target - delay_samples_f);
    feedback += 0.001f * (feedback_target - feedback);
    reverse_mix += 0.001f * (reverse_mix_target - reverse_mix);
    
    delay_samples_f = constrain(delay_samples_f, MIN_DELAY_SAMPLES, MAX_DELAY_SAMPLES);
    rev_len = (int)delay_samples_f;
    if (rev_len < 8) rev_len = 8;
    
    // 判断是否启用倒放
    float current_delay_ms = delay_samples_f * 1000.0f / SAMPLE_RATE;
    reverse_active = (current_delay_ms > 50.0f);
    
    // 正向延迟读取
    float dL = 0.0f, dR = 0.0f;
    float rp = delay_write - delay_samples_f;
    while (rp < 0) rp += MAX_DELAY_SAMPLES;
    
    int i0 = (int)rp;
    int i1 = (i0 + 1) % MAX_DELAY_SAMPLES;
    float frac = rp - i0;
    
    dL = delayL[i0] * (1 - frac) + delayL[i1] * frac;
    dR = delayR[i0] * (1 - frac) + delayR[i1] * frac;
    
    // 写入延迟缓冲区（带反馈）
    delayL[delay_write] = clamp_energy(inL + dL * feedback);
    delayR[delay_write] = clamp_energy(inR + dR * feedback);
    
    // 录制到倒放缓冲区
    if (reverse_active) {
        if (rev_using_A) {
            revA_L[rev_write_pos] = dL;
            revA_R[rev_write_pos] = dR;
        } else {
            revB_L[rev_write_pos] = dL;
            revB_R[rev_write_pos] = dR;
        }
    }
    
    // 处理倒放输出
    float revL = 0.0f, revR = 0.0f;
    
    if (reverse_active && rev_len > 0) {
        // 反向读取
        int reverse_read_idx = rev_len - 1 - rev_read_pos;
        reverse_read_idx = constrain(reverse_read_idx, 0, rev_len - 1);
        
        float* curr_L = rev_using_A ? revB_L : revA_L;
        float* curr_R = rev_using_A ? revB_R : revA_R;
        float* prev_L = rev_using_A ? revA_L : revB_L;
        float* prev_R = rev_using_A ? revA_R : revB_R;
        
        updateFadeGains();
        
        revL = (curr_L[reverse_read_idx] * rev_fade_in_gain) + 
               (prev_L[reverse_read_idx] * rev_fade_out_gain);
        revR = (curr_R[reverse_read_idx] * rev_fade_in_gain) + 
               (prev_R[reverse_read_idx] * rev_fade_out_gain);
        
        // 边界淡出
        float boundary_fade = 1.0f;
        int fade_region = max(rev_len * 0.02f, 16.0f);
        
        if (rev_read_pos < fade_region) {
            boundary_fade = smoothFade((float)rev_read_pos / fade_region);
        } else if ((rev_len - 1 - rev_read_pos) < fade_region) {
            boundary_fade = smoothFade((float)(rev_len - 1 - rev_read_pos) / fade_region);
        }
        
        revL *= boundary_fade;
        revR *= boundary_fade;
        
        // 更新倒放读取位置
        rev_read_pos++;
        if (rev_read_pos >= rev_len) {
            rev_read_pos = 0;
            if (!rev_fading) {
                rev_fading = true;
                rev_fade_pos = 0;
            }
        }
    } else {
        revL = dL;
        revR = dR;
    }
    
    // 混合逻辑
    if (reverse_active) {
        outL = dL * (1.0f - reverse_mix) + revL * reverse_mix;
        outR = dR * (1.0f - reverse_mix) + revR * reverse_mix;
    } else {
        outL = inL * (1.0f - reverse_mix * 0.8f) + dL * reverse_mix * 0.8f;
        outR = inR * (1.0f - reverse_mix * 0.8f) + dR * reverse_mix * 0.8f;
    }
    
    // 更新写指针
    delay_write = (delay_write + 1) % MAX_DELAY_SAMPLES;
    
    // 更新倒放写指针
    if (reverse_active) {
        rev_write_pos++;
        if (rev_write_pos >= rev_len) {
            rev_write_pos = 0;
            rev_using_A = !rev_using_A;
            rev_fading = true;
            rev_fade_pos = 0;
        }
    }
}

/* ================= Audio Processing - Freeze Mode ================= */
void processFreezeMode(float& outL, float& outR) {
    // 使用带符号的速度更新读取指针
    freeze.read_pointer += freeze.speed_current;
    
    // 处理循环，支持正反向
    while (freeze.read_pointer >= MAX_DELAY_SAMPLES)
        freeze.read_pointer -= MAX_DELAY_SAMPLES;
    while (freeze.read_pointer < 0)
        freeze.read_pointer += MAX_DELAY_SAMPLES;
    
    // 读取延迟样本
    int i0 = (int)freeze.read_pointer;
    int i1 = (i0 + 1) % MAX_DELAY_SAMPLES;
    float frac = freeze.read_pointer - i0;
    
    float dL = delayL[i0] * (1 - frac) + delayL[i1] * frac;
    float dR = delayR[i0] * (1 - frac) + delayR[i1] * frac;
    
    // 处理倒放缓冲区
    float revL = dL, revR = dR;
    
    if (freeze.reverse_active && freeze.rev_len > 0) {
        // 倒放读取指针也使用带符号的速度
        freeze.rev_read_pointer += freeze.speed_current;
        
        while (freeze.rev_read_pointer >= freeze.rev_len)
            freeze.rev_read_pointer -= freeze.rev_len;
        while (freeze.rev_read_pointer < 0)
            freeze.rev_read_pointer += freeze.rev_len;
        
        int rev_read_idx = (int)freeze.rev_read_pointer;
        
        // 根据速度方向决定读取方式
        int reverse_read_idx;
        if (freeze.speed_current >= 0) {
            // 正放时正常读取
            reverse_read_idx = rev_read_idx;
        } else {
            // 倒放时反向读取
            reverse_read_idx = freeze.rev_len - 1 - rev_read_idx;
        }
        
        reverse_read_idx = constrain(reverse_read_idx, 0, freeze.rev_len - 1);
        
        float* curr_L = rev_using_A ? revB_L : revA_L;
        float* curr_R = rev_using_A ? revB_R : revA_R;
        float* prev_L = rev_using_A ? revA_L : revB_L;
        float* prev_R = rev_using_A ? revA_R : revB_R;
        
        updateFadeGains();
        
        revL = (curr_L[reverse_read_idx] * rev_fade_in_gain) + 
               (prev_L[reverse_read_idx] * rev_fade_out_gain);
        revR = (curr_R[reverse_read_idx] * rev_fade_in_gain) + 
               (prev_R[reverse_read_idx] * rev_fade_out_gain);
    }
    
    // 根据reverse_mix混合
    if (freeze.reverse_active) {
        outL = dL * (1.0f - freeze.reverse_mix) + revL * freeze.reverse_mix;
        outR = dR * (1.0f - freeze.reverse_mix) + revR * freeze.reverse_mix;
    } else {
        outL = dL;
        outR = dR;
    }
    
    // 更新写指针（循环，不写入新数据）
    delay_write = (delay_write + 1) % MAX_DELAY_SAMPLES;
}

/* ================= Main Loop ================= */
void loop() {
    size_t bytes_read, bytes_written;
    
    processControls();
    
    i2s_read(I2S_PORT, i2s_buf, sizeof(i2s_buf), &bytes_read, portMAX_DELAY);
    int frames = bytes_read / (sizeof(int32_t) * 2);
    
    for (int i = 0; i < frames; i++) {
        float inL = (float)(i2s_buf[i*2] >> 8) / 8388607.0f;
        float inR = (float)(i2s_buf[i*2+1] >> 8) / 8388607.0f;
        
        float outL, outR;
        
        if (freeze.active) {
            processFreezeMode(outL, outR);
        } else {
            processNormalMode(inL, inR, outL, outR);
        }
        
        outL = clamp_energy(outL);
        outR = clamp_energy(outR);
        
        i2s_buf[i*2] = ((int32_t)(outL * 8388607.0f)) << 8;
        i2s_buf[i*2+1] = ((int32_t)(outR * 8388607.0f)) << 8;
    }
    
    i2s_write(I2S_PORT, i2s_buf, bytes_read, &bytes_written, portMAX_DELAY);
}