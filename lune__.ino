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
#define ADC_FEEDBACK     2
#define ADC_REVERSE_MIX  3

/* ================= Audio ================= */
#define SAMPLE_RATE 44100
#define DMA_LEN     256

/* ================= Delay ================= */
#define MAX_DELAY_MS 2000     // 最大2秒
#define MIN_DELAY_MS 20       // 最小20ms
#define MAX_DELAY_SAMPLES (SAMPLE_RATE * MAX_DELAY_MS / 1000)
#define MIN_DELAY_SAMPLES (SAMPLE_RATE * MIN_DELAY_MS / 1000)

/* ================= Reverse ================= */
#define REV_XFADE_MS 30       // 交叉淡化时间
#define REV_XFADE_SAMPLES (SAMPLE_RATE * REV_XFADE_MS / 1000)

#define PIN_FREEZE_BUTTON 11  // Freeze按钮引脚

/* ================= Speed Control ================= */
#define MIN_PLAYBACK_SPEED 0.1f     // 最慢0.2倍
#define MAX_PLAYBACK_SPEED 5.0f     // 最快5倍

/* ================= Buffers ================= */
static float *delayL, *delayR;
static float *revA_L, *revA_R;
static float *revB_L, *revB_R;
static int32_t i2s_buf[DMA_LEN * 2];

/* ================= 相位检测辅助函数 ================= */
// 声明缓冲区指针为外部变量，让PhaseState可以访问
extern float *delayL, *delayR;

// 计算当前播放的相位（0-1）
float calculateCurrentPhase(float current_delay, int delay_write_pos, float* delayL_buf, float* delayR_buf) {
    // 查找最近的声音能量峰值
    static float last_peak_pos = 0.0f;
    
    // 简单的峰值检测：找最近的128个样本
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
        // 没有明显信号，保持上次的相位
        return last_peak_pos;
    }
    
    // 计算相位（距离写指针多远，归一化到0-1）
    float distance = (delay_write_pos - peak_idx + MAX_DELAY_SAMPLES) % MAX_DELAY_SAMPLES;
    float phase = distance / current_delay;
    
    // 平滑相位变化
    if (fabs(phase - last_peak_pos) < 0.1f || last_peak_pos == 0.0f) {
        last_peak_pos = phase;
    }
    
    return last_peak_pos;
}

/* ================= Freeze参数 ================= */
struct FreezeState {
    bool active = false;
    float base_delay = 0.0f;
    float feedback = 0.0f;
    float reverse_mix = 0.0f;
    bool reverse_active = false;
    int rev_len = 0;
    
    // 速度控制
    float base_ads = 0.0f;
    float speed_target = 1.0f;
    float speed_current = 1.0f;
    float read_pointer = 0.0f;
    float rev_read_pointer = 0.0f;
    
    // 新增：相位同步相关
    float freeze_phase = 0.0f;      // 冻结瞬间的相位
    float freeze_instant_speed = 1.0f; // 冻结瞬间的实际速度
    
    // 检测冻结瞬间的播放速度（简化版本）
    float detectFreezeSpeed(float current_delay, int delay_write_pos) {
        // 简化方法：通过最近几个样本的能量变化估算速度
        static float last_avg_energy = 0.0f;
        static uint32_t last_time = 0;
        
        // 计算当前的平均能量
        float current_avg_energy = 0.0f;
        for (int i = 0; i < 16; i++) {
            int idx = (delay_write_pos - i + MAX_DELAY_SAMPLES) % MAX_DELAY_SAMPLES;
            current_avg_energy += fabs(delayL[idx]) + fabs(delayR[idx]);
        }
        current_avg_energy /= 16.0f;
        
        uint32_t now = millis();
        float detected_speed = 1.0f; // 默认正常速度
        
        if (last_time > 0 && now > last_time) {
            float energy_diff = current_avg_energy - last_avg_energy;
            float time_diff = (now - last_time) / 1000.0f; // 秒
            
            // 能量变化率可以粗略反映播放速度
            if (time_diff > 0.01f) { // 至少10ms
                float energy_rate = fabs(energy_diff) / time_diff;
                
                // 经验映射：能量变化率 -> 播放速度
                if (energy_rate < 0.01f) detected_speed = 0.5f; // 慢速
                else if (energy_rate < 0.05f) detected_speed = 1.0f; // 正常
                else if (energy_rate < 0.1f) detected_speed = 1.5f; // 快速
                else detected_speed = 2.0f; // 很快
                
                // 限制范围
                detected_speed = constrain(detected_speed, MIN_PLAYBACK_SPEED, MAX_PLAYBACK_SPEED);
            }
        }
        
        last_avg_energy = current_avg_energy;
        last_time = now;
        
        return detected_speed;
    }
    
    void init(float delay, float fb, float mix, bool rev, int rlen, float ads_val, 
              int delay_write_pos) {
        base_delay = delay;
        feedback = fb;
        reverse_mix = mix;
        reverse_active = rev;
        rev_len = rlen;
        base_ads = ads_val;
        
        // 检测冻结瞬间的播放速度
        freeze_instant_speed = detectFreezeSpeed(delay, delay_write_pos);
        
        // 计算当前播放相位
        freeze_phase = calculateCurrentPhase(delay, delay_write_pos, delayL, delayR);
        
        // 关键改进：冻结瞬间的读取指针要与当前播放相位同步
        read_pointer = delay_write_pos - (freeze_phase * base_delay);
        if (read_pointer < 0) read_pointer += MAX_DELAY_SAMPLES;
        if (read_pointer >= MAX_DELAY_SAMPLES) read_pointer -= MAX_DELAY_SAMPLES;
        
        // 计算目标速度：基于ADS值和当前实际速度
        float norm = constrain(ads_val, 0, 26000) / 26000.0f;
        speed_target = freeze_instant_speed * (MAX_PLAYBACK_SPEED - (norm * (MAX_PLAYBACK_SPEED - MIN_PLAYBACK_SPEED)));
        speed_current = freeze_instant_speed; // 初始速度设置为冻结瞬间的实际速度
        
        rev_read_pointer = 0.0f;
        
        Serial.printf("Freeze Init: Phase=%.3f, Instant Speed=%.3fx, Target Speed=%.3fx\n", 
                     freeze_phase, freeze_instant_speed, speed_target);
    }
    
    void updateSpeed(int16_t ads_raw) {
        float norm = constrain(ads_raw, 0, 26000) / 26000.0f;
        // 相对变化：在冻结瞬间速度的基础上调整
        float relative_speed = MAX_PLAYBACK_SPEED - (norm * (MAX_PLAYBACK_SPEED - MIN_PLAYBACK_SPEED));
        speed_target = freeze_instant_speed * relative_speed;
        
        // 平滑过渡
        speed_current += 0.05f * (speed_target - speed_current);
        speed_current = constrain(speed_current, MIN_PLAYBACK_SPEED, MAX_PLAYBACK_SPEED);
    }
};

static FreezeState freeze;

/* ================= State ================= */
static int delay_write = 0;
static float delay_samples_f = 8000;
static float delay_target = 8000;
static float feedback = 0.35f;
static float feedback_target = 0.35f;

/* ================= Reverse State ================= */
static int rev_len = 8000;
static int rev_write_pos = 0;
static int rev_read_pos = 0;
static bool rev_using_A = true;
static bool reverse_active = false;

/* ================= Crossfade ================= */
static bool rev_fading = false;
static int rev_fade_pos = 0;
static float rev_fade_in_gain = 0.0f;
static float rev_fade_out_gain = 1.0f;

/* ================= Mix ================= */
static float reverse_mix = 0.0f;
static float reverse_mix_target = 0.0f;

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
    Serial.println("=== Reverse Delay with Phase-Sync Freeze ===");
    
    // 初始化引脚
    pinMode(PIN_FREEZE_BUTTON, INPUT_PULLUP);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(ADC_FEEDBACK, INPUT);
    pinMode(ADC_REVERSE_MIX, INPUT);
    
    // 初始化I2C和ADS1115
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!ads.begin()) {
        Serial.println("Failed to initialize ADS1115!");
        while(1);
    }
    ads.setGain(GAIN_ONE);
    ads.setDataRate(RATE_ADS1115_860SPS);
    
    // 分配内存
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
    
    // 初始化缓冲区
    for (int i = 0; i < MAX_DELAY_SAMPLES; i++) {
        delayL[i] = (rand() % 200 - 100) / 1000000.0f;
        delayR[i] = (rand() % 200 - 100) / 1000000.0f;
        revA_L[i] = (rand() % 200 - 100) / 1000000.0f;
        revA_R[i] = (rand() % 200 - 100) / 1000000.0f;
        revB_L[i] = (rand() % 200 - 100) / 1000000.0f;
        revB_R[i] = (rand() % 200 - 100) / 1000000.0f;
    }
    
    // 配置I2S
    i2s_config_t cfg = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
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
    
    Serial.println("System ready! Freeze with phase synchronization enabled.");
}

/* ================= Control Processing ================= */
void processControls() {
    static uint32_t last_ctrl_time = 0;
    static bool last_freeze_btn_state = HIGH;
    static bool btn_debounce_flag = false;
    
    if (millis() - last_ctrl_time < 15) return;
    
    // 读取ADS1115
    int16_t raw = ads.readADC_SingleEnded(0);
    raw = constrain(raw, 0, 26000);
    
    // Freeze按钮处理
    bool curr_btn_state = digitalRead(PIN_FREEZE_BUTTON);
    
    if (curr_btn_state == LOW && last_freeze_btn_state == HIGH && !btn_debounce_flag) {
        freeze.active = !freeze.active;
        btn_debounce_flag = true;
        
        if (freeze.active) {
            // ===== 关键改进：冻结瞬间捕获当前播放状态 =====
            // 初始化冻结状态（包含相位和速度信息）
            freeze.init(delay_samples_f, feedback, reverse_mix, 
                       reverse_active, rev_len, (float)raw,
                       delay_write);
            
            Serial.printf("Freeze ON | Phase=%.3f | Speed=%.3fx | Delay=%.1f\n", 
                         freeze.freeze_phase, freeze.freeze_instant_speed, delay_samples_f);
        } else {
            Serial.println("Freeze OFF");
        }
    }
    
    if (curr_btn_state == HIGH && last_freeze_btn_state == LOW) {
        btn_debounce_flag = false;
    }
    last_freeze_btn_state = curr_btn_state;
    
    // 更新控制目标
    if (!freeze.active) {
        // 正常模式：所有参数受控
        delay_target = MIN_DELAY_SAMPLES + (float)raw * (MAX_DELAY_SAMPLES - MIN_DELAY_SAMPLES) / 26000.0f;
        feedback_target = constrain(analogRead(ADC_FEEDBACK) / 4095.0f, 0.0f, 0.90f);
        reverse_mix_target = constrain(analogRead(ADC_REVERSE_MIX) / 4095.0f, 0.0f, 1.0f);
    } else {
        // 冻结模式：只更新速度，其他参数锁定
        freeze.updateSpeed(raw);
        delay_target = freeze.base_delay;
        feedback_target = freeze.feedback;
        reverse_mix_target = freeze.reverse_mix;
    }
    
    last_ctrl_time = millis();
}

/* ================= Audio Processing - Normal Mode ================= */
void processNormalMode(int frame_index, float inL, float inR, 
                      float& outL, float& outR) {
    // 平滑参数变化
    delay_samples_f += 0.0005f * (delay_target - delay_samples_f);
    feedback += 0.001f * (feedback_target - feedback);
    reverse_mix += 0.001f * (reverse_mix_target - reverse_mix);
    
    delay_samples_f = constrain(delay_samples_f, MIN_DELAY_SAMPLES, MAX_DELAY_SAMPLES);
    rev_len = (int)delay_samples_f;
    if (rev_len < 8) rev_len = 8;
    
    // 控制reverse是否激活
    float current_delay_ms = delay_samples_f * 1000.0f / SAMPLE_RATE;
    reverse_active = (current_delay_ms > 50.0f);
    
    // 正向延迟处理
    float dL = 0.0f, dR = 0.0f;
    
    if (delay_samples_f > 10.0f) {
        float rp = delay_write - delay_samples_f;
        if (rp < 0) rp += MAX_DELAY_SAMPLES;
        if (rp >= MAX_DELAY_SAMPLES) rp -= MAX_DELAY_SAMPLES;
        
        int i0 = (int)rp;
        int i1 = (i0 + 1) % MAX_DELAY_SAMPLES;
        float frac = rp - i0;
        
        dL = delayL[i0] * (1.0f - frac) + delayL[i1] * frac;
        dR = delayR[i0] * (1.0f - frac) + delayR[i1] * frac;
        
        // 写入延迟缓冲区
        delayL[delay_write] = clamp_energy(inL + dL * feedback);
        delayR[delay_write] = clamp_energy(inR + dR * feedback);
    } else {
        dL = inL;
        dR = inR;
        delayL[delay_write] = inL;
        delayR[delay_write] = inR;
    }
    
    // 录制到倒放缓冲区
    if (reverse_active && rev_write_pos < MAX_DELAY_SAMPLES) {
        float* wL = rev_using_A ? revA_L : revB_L;
        float* wR = rev_using_A ? revA_R : revB_R;
        wL[rev_write_pos] = dL;
        wR[rev_write_pos] = dR;
    }
    
    // 处理倒放
    float revL = 0.0f, revR = 0.0f;
    
    if (reverse_active && rev_len > 0) {
        rev_read_pos = constrain(rev_read_pos, 0, rev_len - 1);
        int reverse_read_idx = rev_len - 1 - rev_read_pos;
        
        reverse_read_idx = constrain(reverse_read_idx, 0, min(rev_len-1, MAX_DELAY_SAMPLES-1));
        
        float* curr_L = rev_using_A ? revB_L : revA_L;
        float* curr_R = rev_using_A ? revB_R : revA_R;
        float* prev_L = rev_using_A ? revA_L : revB_L;
        float* prev_R = rev_using_A ? revA_R : revB_R;
        
        updateFadeGains();
        
        float curr_sample_L = curr_L[reverse_read_idx];
        float curr_sample_R = curr_R[reverse_read_idx];
        float prev_sample_L = prev_L[reverse_read_idx];
        float prev_sample_R = prev_R[reverse_read_idx];
        
        revL = (curr_sample_L * rev_fade_in_gain) + (prev_sample_L * rev_fade_out_gain);
        revR = (curr_sample_R * rev_fade_in_gain) + (prev_sample_R * rev_fade_out_gain);
        
        // 边界淡出
        float boundary_fade = 1.0f;
        int samples_from_start = rev_read_pos;
        int samples_from_end = rev_len - 1 - rev_read_pos;
        float fade_region = max(rev_len * 0.02f, 16.0f);
        
        if (samples_from_start < fade_region) {
            boundary_fade = smoothFade((float)samples_from_start / fade_region);
        } else if (samples_from_end < fade_region) {
            boundary_fade = smoothFade((float)samples_from_end / fade_region);
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
    
    // 混合输出
    if (reverse_active) {
        outL = inL * (1.0f - reverse_mix) + revL * reverse_mix;
        outR = inR * (1.0f - reverse_mix) + revR * reverse_mix;
    } else {
        if (reverse_mix < 0.5f) {
            float delay_mix = 1.0f - (reverse_mix * 2.0f);
            outL = inL * delay_mix + dL * (1.0f - delay_mix);
            outR = inR * delay_mix + dR * (1.0f - delay_mix);
        } else {
            float raw_mix = (reverse_mix - 0.5f) * 2.0f;
            outL = inL * (1.0f - raw_mix) + dL * raw_mix;
            outR = inR * (1.0f - raw_mix) + dR * raw_mix;
        }
    }
    
    // 更新位置指针
    delay_write = (delay_write + 1) % MAX_DELAY_SAMPLES;
    
    if (reverse_active) {
        rev_write_pos++;
        if (rev_write_pos >= rev_len) {
            rev_write_pos = 0;
            rev_using_A = !rev_using_A;
        }
    }
}

/* ================= Audio Processing - Freeze Mode ================= */
void processFreezeMode(int frame_index, float& outL, float& outR) {
    // 更新读取指针（带速度控制）
    freeze.read_pointer += freeze.speed_current;
    if (freeze.read_pointer >= MAX_DELAY_SAMPLES) {
        freeze.read_pointer -= MAX_DELAY_SAMPLES;
    }
    
    // 读取延迟样本
    float dL, dR;
    if (freeze.base_delay > 10.0f) {
        // 关键改进：使用相位同步的读取方式
        float rp = freeze.read_pointer;
        
        int i0 = (int)rp;
        int i1 = (i0 + 1) % MAX_DELAY_SAMPLES;
        float frac = rp - i0;
        
        dL = delayL[i0] * (1.0f - frac) + delayL[i1] * frac;
        dR = delayR[i0] * (1.0f - frac) + delayR[i1] * frac;
    } else {
        int idx = (int)freeze.read_pointer % MAX_DELAY_SAMPLES;
        dL = delayL[idx];
        dR = delayR[idx];
    }
    
    // 处理倒放
    float revL = 0.0f, revR = 0.0f;
    
    if (freeze.reverse_active && freeze.rev_len > 0) {
        // 倒放缓冲区也需要相位同步
        freeze.rev_read_pointer += freeze.speed_current;
        if (freeze.rev_read_pointer >= freeze.rev_len) {
            freeze.rev_read_pointer = 0;
            if (!rev_fading) {
                rev_fading = true;
                rev_fade_pos = 0;
            }
        }
        
        int rev_read_idx = constrain((int)freeze.rev_read_pointer, 0, freeze.rev_len - 1);
        int reverse_read_idx = freeze.rev_len - 1 - rev_read_idx;
        reverse_read_idx = constrain(reverse_read_idx, 0, min(freeze.rev_len-1, MAX_DELAY_SAMPLES-1));
        
        float* curr_L = rev_using_A ? revB_L : revA_L;
        float* curr_R = rev_using_A ? revB_R : revA_R;
        float* prev_L = rev_using_A ? revA_L : revB_L;
        float* prev_R = rev_using_A ? revA_R : revB_R;
        
        updateFadeGains();
        
        float curr_sample_L = curr_L[reverse_read_idx];
        float curr_sample_R = curr_R[reverse_read_idx];
        float prev_sample_L = prev_L[reverse_read_idx];
        float prev_sample_R = prev_R[reverse_read_idx];
        
        revL = (curr_sample_L * rev_fade_in_gain) + (prev_sample_L * rev_fade_out_gain);
        revR = (curr_sample_R * rev_fade_in_gain) + (prev_sample_R * rev_fade_out_gain);
    } else {
        revL = dL;
        revR = dR;
    }
    
    // 输出混合
    if (freeze.reverse_active) {
        outL = revL * freeze.reverse_mix;
        outR = revR * freeze.reverse_mix;
    } else {
        outL = dL;
        outR = dR;
    }
    
    // 更新延迟写入指针（循环，不写入新数据）
    delay_write = (delay_write + 1) % MAX_DELAY_SAMPLES;
}

/* ================= Main Loop ================= */
void loop() {
    size_t bytes_read, bytes_written;
    
    // 处理控制输入
    processControls();
    
    // 读取音频输入
    i2s_read(I2S_PORT, i2s_buf, sizeof(i2s_buf), &bytes_read, portMAX_DELAY);
    int frames = bytes_read / (sizeof(int32_t) * 2);
    
    for (int i = 0; i < frames; i++) {
        float inL = 0.0f, inR = 0.0f;
        float outL, outR;
        
        // 转换输入信号
        if (!freeze.active) {
            inL = (float)(i2s_buf[i*2] >> 8) / 8388607.0f;
            inR = (float)(i2s_buf[i*2+1] >> 8) / 8388607.0f;
        }
        
        if (freeze.active) {
            processFreezeMode(i, outL, outR);
        } else {
            processNormalMode(i, inL, inR, outL, outR);
        }
        
        // 输出限幅和增益
        outL = clamp_energy(outL * 0.9f);
        outR = clamp_energy(outR * 0.9f);
        
        // 转换回24位格式
        i2s_buf[i*2]   = ((int32_t)(outL * 8388607.0f)) << 8;
        i2s_buf[i*2+1] = ((int32_t)(outR * 8388607.0f)) << 8;
    }
    
    // 输出音频
    i2s_write(I2S_PORT, i2s_buf, bytes_read, &bytes_written, portMAX_DELAY);
}