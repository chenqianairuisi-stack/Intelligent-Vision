warning: in the working copy of 'project/mdk/rt1064.uvoptx', LF will be replaced by CRLF the next time Git touches it
diff --git a/project/App/GameManage.cpp b/project/App/GameManage.cpp
index fe5034f..516392e 100644
--- a/project/App/GameManage.cpp
+++ b/project/App/GameManage.cpp
@@ -76,7 +76,7 @@ __attribute__((section(".ramfunc"))) void GameManager::update() {
     switch (game.phase) {
         case GamePhase::INIT_CALIBRATE: {
             // 灏嗛噷绋嬭閲嶇疆鍒板凡鐭ュ叆鍙ｄ綅濮�
-            Subsystem::PoseEstimator::set_position(ENTRY_X, ENTRY_Y);
+            Subsystem::PoseEstimator::set_position(ENTRY_X, ENTRY_Y, ENTRY_YAW);
             
             // 鐩存帴涓嬪彂鍑哄簱鐩爣鐐癸紝鍒囧埌鎵嬪姩鐩爣妯″紡鎵ц绂诲満
             ctrl.current_target = {OUT_TARGET_X, OUT_TARGET_Y, ENTRY_YAW};
diff --git a/project/App/main.cpp b/project/App/main.cpp
index 2e3eb2a..641c64b 100644
--- a/project/App/main.cpp
+++ b/project/App/main.cpp
@@ -8,6 +8,7 @@
 #include "Display.h"
 #include "PoseEstimate.h"
 #include "Storage.h"
+#include "system_config.h"
 
 
 extern "C" int main(void) {
@@ -31,8 +32,8 @@ extern "C" int main(void) {
     system_delay_ms(500);
     Subsystem::PoseEstimator::calibrate_gyro_step();
 
-    pit_ms_init(PIT_CH0, 5);                 
-    pit_ms_init(PIT_CH1, 20);               
+    pit_ms_init(PIT_CH0, SystemConfig::PIT_CH0_PERIOD_MS);
+    pit_ms_init(PIT_CH1, SystemConfig::PIT_CH1_PERIOD_MS);
     interrupt_set_priority(PIT_IRQn, 0);    
     interrupt_global_enable(0);
     
diff --git a/project/Core/isr.cpp b/project/Core/isr.cpp
index f7bcf92..9cfe702 100644
--- a/project/Core/isr.cpp
+++ b/project/Core/isr.cpp
@@ -12,16 +12,19 @@
 
 extern "C" void PIT_IRQHandler(void) {
 
-    // 5ms 锟斤拷时锟斤拷锟叫断ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟捷讹拷取锟斤拷锟斤拷锟�
+    // PIT_CH0 锟斤拷时锟斤拷锟叫断ｏ拷锟斤拷锟斤拷锟斤拷 SystemConfig::PIT_CH0_PERIOD_MS 锟斤拷锟矫ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟捷讹拷取锟斤拷锟斤拷锟�
     if(pit_flag_get(PIT_CH0)) 
     {
         pit_flag_clear(PIT_CH0);
 
-        imu_icm42688.update_gyro_only();                   // 锟斤拷锟斤拷锟斤拷锟斤拷锟捷讹拷取锟斤拷转锟斤拷
-        Subsystem::PoseEstimator::update_yaw_5ms_tick();   // yaw 锟斤拷嵌雀锟斤拷锟�
+        // 锟斤拷锟斤拷锟斤拷锟斤拷锟捷讹拷取锟斤拷转锟斤拷
+        imu_icm42688.update_all();  
+
+        // yaw 锟斤拷嵌雀锟斤拷锟�
+        Subsystem::PoseEstimator::update_yaw_1ms_tick();  
     }
     
-    // 20ms 锟斤拷时锟斤拷锟叫断ｏ拷锟斤拷锟节碉拷锟教匡拷锟斤拷锟姐法锟斤拷锟铰猴拷锟斤拷碳锟斤拷锟斤拷锟�
+    // PIT_CH1 锟斤拷时锟斤拷锟叫断ｏ拷锟斤拷锟斤拷锟斤拷 SystemConfig::PIT_CH1_PERIOD_MS 锟斤拷锟矫ｏ拷锟斤拷锟斤拷锟节碉拷锟教匡拷锟斤拷锟姐法锟斤拷锟铰猴拷锟斤拷碳锟斤拷锟斤拷锟�
     if(pit_flag_get(PIT_CH1)) 
     {
         pit_flag_clear(PIT_CH1);
diff --git a/project/Core/system_config.h b/project/Core/system_config.h
index 384babc..34f4010 100644
--- a/project/Core/system_config.h
+++ b/project/Core/system_config.h
@@ -11,6 +11,12 @@
 // 鍏ㄥ眬绯荤粺閰嶇疆鍜屽父閲忓畾涔�
 // =================================================================
 namespace SystemConfig {
+    // 瀹氭椂鍣ㄥ弬鏁�
+    static constexpr uint32_t PIT_CH0_PERIOD_MS = 1U;          // IMU 閲囨牱涓庡Э 鎬佽В绠楀懆鏈�
+    static constexpr uint32_t PIT_CH1_PERIOD_MS = 20U;         // 搴曠洏鎺у埗涓庨噷 绋嬭鍛ㄦ湡
+    static constexpr float PIT_CH0_DT_S = static_cast<float>(PIT_CH0_PERIOD_MS) * 0.001f;  // 杞崲涓虹
+    static constexpr float PIT_CH1_DT_S = static_cast<float>(PIT_CH1_PERIOD_MS) * 0.001f;  // 杞崲涓虹
+
     // 鏈烘鍙傛暟
     static constexpr float WHEEL_RADIUS = 3.15f;                // 杞瓙鍗婂緞锛屽崟浣嶏細鍘樼背
     static constexpr float HALF_X_AXIS = 9.0f;                  // x 杞村崐杞磋窛锛屽崟浣嶏細鍘樼背
@@ -50,7 +56,8 @@ namespace SystemConfig {
     static constexpr float OUT_TARGET_Y = 30.0f;                // 鍑哄簱鐩爣浣嶇疆 Y 鍧愭爣
 
     // 鏁板甯告暟
-    static constexpr float DEG_TO_RAD = 3.1415926535f / 180.0f;
+    static constexpr float DEG_TO_RAD = 0.017453292519943f;
+    static constexpr float RAD_TO_DEG = 57.29577951308232f;
 }
 
 
diff --git a/project/Core/tuning_config.h b/project/Core/tuning_config.h
index cfac8e2..d9070a8 100644
--- a/project/Core/tuning_config.h
+++ b/project/Core/tuning_config.h
@@ -44,16 +44,16 @@ struct TuningConfig {
 
 // 鍏ㄥ眬璋冨弬瀹炰緥锛屾斁鍦� DTCM 鍖哄煙锛屼緵鎵€鏈夋ā鍧楄闂�
 __attribute__((section(".dtcm_data"))) inline TuningConfig tune {
-    {9.0f, 0.5f, 0.6f},         // pid_yaw
-    {0.2f, 0.1f, 0.0f},         // pid_speed
-    {0.1f, 0.0f},               // feedforward
+    {4.5f, 0.0f, 0.4f},         // pid_yaw
+    {0.3f, 0.1f, 0.0f},         // pid_speed
+    {0.15f, 0.0f},               // feedforward
 
     // Dynamics 鍔ㄥ姏瀛﹂娴嬪弬鏁�
     {
-        65.0f,     // max_duty
-        120.0f,    // max_speed
-        80.0f,     // max_acc
-        1200.0f,   // max_jerk
+        60.0f,     // max_duty
+        80.0f,    // max_speed
+        60.0f,     // max_acc
+        1000.0f,   // max_jerk
         3.0f,      // max_ang_speed
         1.03f,     // kinematic_gain_x
         1.01f      // kinematic_gain_y
diff --git a/project/Device/encoder.h b/project/Device/encoder.h
index b884972..afa397e 100644
--- a/project/Device/encoder.h
+++ b/project/Device/encoder.h
@@ -8,15 +8,15 @@ public:
        EncoderArray() = default;
        void init();
 
-       // 20ms 涓柇璋冪敤锛屾洿鏂板閲忚鏁板€硷紝骞惰浆鎹负閫熷害鏇存柊鍏ㄥ眬鐘舵€�
+       // PIT_CH1 涓柇璋冪敤锛屾洿鏂板閲忚鏁板€硷紝骞惰浆鎹负閫熷害鏇存柊鍏ㄥ眬鐘舵€�
        void update_encoders_20ms_tick();
 
        // 鑾峰彇鎵€鏈夌紪鐮佸櫒璁℃暟鐨勬寚閽堬紝渚涘閮ㄤ娇鐢� (娉細杩斿洖鐨勬槸澧為噺璁℃暟鍊�)
        const int16_t* getAllCounts() const { return counts; }      
 
 private:
-       // 浠庣紪鐮佸櫒鑴夊啿杞崲鍒� cm/s 鐨勭郴鏁� (鍋囪 20ms 鏇存柊涓€娆�)
-       static constexpr float PULSES_TO_SPEED_CM_S = (2.0f * PI * SystemConfig::WHEEL_RADIUS) / (SystemConfig::PULSES_PER_REV * 0.02f);  
+       // 浠庣紪鐮佸櫒鑴夊啿杞崲鍒� cm/s 鐨勭郴鏁� (鎸� PIT_CH1_DT_S 璁＄畻)
+       static constexpr float PULSES_TO_SPEED_CM_S = (2.0f * PI * SystemConfig::WHEEL_RADIUS) / (SystemConfig::PULSES_PER_REV * SystemConfig::PIT_CH1_DT_S);  
 
        int16_t counts[4] = {0, 0, 0, 0};                           // 褰撳墠鍛ㄦ湡 鐨勫閲忚鏁板€� (椤哄簭 LF, LB, RF, RB, 宸蹭箻涓婃瀬鎬�)
        int32_t last_raw[4] = {0, 0, 0, 0};                         // 纭欢瀹氭椂 鍣ㄤ笂涓€娆＄殑缁濆璁℃暟鍊� (椤哄簭 LF, LB, RF, RB)
diff --git a/project/Device/icm42688.cpp b/project/Device/icm42688.cpp
index e54c346..2a7217d 100644
--- a/project/Device/icm42688.cpp
+++ b/project/Device/icm42688.cpp
@@ -70,13 +70,13 @@ __attribute__((section(".ramfunc"))) void Icm42688::update_all() {
     data.raw_gyro_y = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
     data.raw_gyro_z = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);
 
-    // 涔樹互棰勭紪璇戠殑甯告暟杞崲涓虹墿鐞嗛噺
+    // 涔樹互棰勭紪璇戠殑甯告暟杞崲涓虹墿鐞嗛噺,骞朵笖璋冩暣纭欢鍧愭爣绯�
     data.temp   = static_cast<float>(data.raw_temp) * TEMP_SCALE + TEMP_OFFSET;
-    data.acc_x  = static_cast<float>(data.raw_acc_x) * ACCEL_SCALE;
-    data.acc_y  = static_cast<float>(data.raw_acc_y) * ACCEL_SCALE;
-    data.acc_z  = static_cast<float>(data.raw_acc_z) * ACCEL_SCALE;
-    data.gyro_x = static_cast<float>(data.raw_gyro_x) * GYRO_SCALE;
-    data.gyro_y = static_cast<float>(data.raw_gyro_y) * GYRO_SCALE;
+    data.acc_x  = static_cast<float>(data.raw_acc_y) * ACCEL_SCALE;
+    data.acc_y  = static_cast<float>(data.raw_acc_x) * ACCEL_SCALE;
+    data.acc_z  = - static_cast<float>(data.raw_acc_z) * ACCEL_SCALE;
+    data.gyro_x = - static_cast<float>(data.raw_gyro_y) * GYRO_SCALE;
+    data.gyro_y = - static_cast<float>(data.raw_gyro_x) * GYRO_SCALE;
     data.gyro_z = - static_cast<float>(data.raw_gyro_z) * GYRO_SCALE;
 }
 
diff --git a/project/Subsystem/ChassisControl.cpp b/project/Subsystem/ChassisControl.cpp
index 7aa4316..9cfba81 100644
--- a/project/Subsystem/ChassisControl.cpp
+++ b/project/Subsystem/ChassisControl.cpp
@@ -40,14 +40,14 @@ namespace {
     __attribute__((always_inline)) inline float normalize_angle(float angle) {
         if (angle > 180.0f)       angle -= 360.0f;
         else if (angle < -180.0f) angle += 360.0f;
-        return angle * 3.1415926535f / 180.0f; 
+        return angle * SystemConfig::DEG_TO_RAD;
     }
 
     // 鎻愬彇绗﹀彿锛岀敤浜庨潤鎽╂摝鍓嶉鏂瑰悜鍒ゆ柇锛屽苟寮曞叆姝诲尯闃叉闆剁偣闇囪崱
     __attribute__((always_inline)) inline float get_sign(float val) {
-        if (val > 0.5f) return 1.0f;
-        if (val < -0.5f) return -1.0f;
-        return val / 0.5f; // 鍦� -0.5 鍒� 0.5 涔嬮棿锛屽钩婊戝湴浠� -1 杩囨浮鍒� 1锛岀粷涓嶇獊鍙�
+        if (val > 1.0f) return 1.0f;
+        if (val < -1.0f) return -1.0f;
+        return val / 1.0f; // 鍦� -1.0 鍒� 1.0 涔嬮棿锛屽钩婊戝湴浠� -1 杩囨浮鍒� 1锛岀粷涓嶇獊鍙�
     }
 
 
@@ -102,7 +102,7 @@ __attribute__((section(".ramfunc"))) void update_20ms_tick() {
     float err_yaw = normalize_angle(ctrl.current_target.yaw - yaw);
 
     // 閫熷害瑙勫垝
-    Speed2D expected_global_vel = velocity_planner.velocity_planning_1d(err_global_x, err_global_y, 0.02f);
+    Speed2D expected_global_vel = velocity_planner.velocity_planning_1d(err_global_x, err_global_y, SystemConfig::PIT_CH1_DT_S);
 
     // 灏嗗叏灞€鏈熸湜閫熷害鎶曞奖鍒板皬杞﹁嚜韬殑灞€閮ㄥ潗鏍囩郴
     float current_yaw_rad = yaw * SystemConfig::DEG_TO_RAD;  // 杞崲涓哄姬搴�
@@ -146,8 +146,8 @@ __attribute__((section(".ramfunc"))) void check_is_stopped() {
         
     // 鍒ゅ畾鏉′欢锛氫笂涓€甯х殑鎺у埗鐩爣鍑犱箮涓�0锛屼笖褰撳墠鍥涗釜杞瓙鐨勭湡瀹炲弽棣堥€熷害鏋佸皬
     App::g_state.physical.is_stopped = 
-        (std::abs(cur_spd.lf) < 0.5f && std::abs(cur_spd.lb) < 0.5f &&
-         std::abs(cur_spd.rf) < 0.5f && std::abs(cur_spd.rb) < 0.5f);
+        (std::abs(cur_spd.lf) < 0.2f && std::abs(cur_spd.lb) < 0.2f &&
+         std::abs(cur_spd.rf) < 0.2f && std::abs(cur_spd.rb) < 0.2f);
 }
 
 }
diff --git a/project/Subsystem/ChassisControl.h b/project/Subsystem/ChassisControl.h
index b5edcfa..16ddfae 100644
--- a/project/Subsystem/ChassisControl.h
+++ b/project/Subsystem/ChassisControl.h
@@ -2,7 +2,8 @@
 
 namespace Subsystem::Chassis {
     void init();
-    void update_20ms_tick();  // 鏀惧埌 20ms 瀹氭椂鍣ㄤ腑鏂噷
     void check_is_stopped();
+
+    void update_20ms_tick();  // PIT_CH1 瀹氭椂鍣ㄨ皟鐢紝鎵ц搴曠洏鎺у埗绠楁硶鏇存柊
     void update_20ms_tick_debug();
 }
\ No newline at end of file
diff --git a/project/Subsystem/Display.cpp b/project/Subsystem/Display.cpp
index d3feed2..846a853 100644
--- a/project/Subsystem/Display.cpp
+++ b/project/Subsystem/Display.cpp
@@ -4,6 +4,7 @@
 #include "GameManage.h"
 #include "Storage.h"
 #include "Encoder.h"
+#include "Icm42688.h"
 
 // 纭欢寮曡剼瀹氫箟
 #define KEY1 C13  // 涓嬬Щ / 鍑忓皬
@@ -63,8 +64,8 @@ namespace { // 鍖垮悕鍛藉悕绌洪棿锛岀‘淇濊繖浜涙暟鎹彧鍦ㄦ湰鏂囦欢鍙
     {"Max_Acc ",   &tune.dynamics.max_acc,            5.0f  },
     {"MaxJerk ",   &tune.dynamics.max_jerk,           50.0f },
     {"MaxASpd ",   &tune.dynamics.max_ang_speed,      0.1f  },
-    {"Gain_X  ",   &tune.dynamics.kinematic_gain_x,   0.01f },
-    {"Gain_Y  ",   &tune.dynamics.kinematic_gain_y,   0.01f },
+    {"Gain_X  ",   &tune.dynamics.kinematic_gain_x,   0.001f },
+    {"Gain_Y  ",   &tune.dynamics.kinematic_gain_y,   0.001f },
     {"Reach_R ",   &tune.tracker.reach_radius,        0.1f  },
     {"Reach_M ",   &tune.tracker.reach_radius_min,    0.1f  }
      };
@@ -394,12 +395,19 @@ void draw_odometry_data() {
 
     tft180_show_string(0, 2 * UI_ROW_H, "Global X: ");   tft180_show_float(10 * UI_COL_W, 2 * UI_ROW_H, pos.x, 3, 1);
     tft180_show_string(0, 3 * UI_ROW_H, "Global Y: ");   tft180_show_float(10 * UI_COL_W, 3 * UI_ROW_H, pos.y, 3, 1);
-    char ui_buf[32]; sprintf(ui_buf, "Yaw: %8.2f   ", pos.yaw);     tft180_show_string(0, 4 * UI_ROW_H, ui_buf);
-
-    tft180_show_string(0, 5 * UI_ROW_H, "Spd LF: ");     tft180_show_float(10 * UI_COL_W, 5 * UI_ROW_H, wheels.lf, 3, 1);
-    tft180_show_string(0, 6 * UI_ROW_H, "Spd LB: ");     tft180_show_float(10 * UI_COL_W, 6 * UI_ROW_H, wheels.lb, 3, 1);
-    tft180_show_string(0, 7 * UI_ROW_H, "Spd RF: ");     tft180_show_float(10 * UI_COL_W, 7 * UI_ROW_H, wheels.rf, 3, 1);
-    tft180_show_string(0, 8 * UI_ROW_H, "Spd RB: ");     tft180_show_float(10 * UI_COL_W, 8 * UI_ROW_H, wheels.rb, 3, 1);
+    char ui_buf[32]; 
+    sprintf(ui_buf, "Yaw: %8.2f   ", pos.yaw);     tft180_show_string(0, 4 * UI_ROW_H, ui_buf);
+    sprintf(ui_buf, "Acc_Z: %8.2f   ", imu_icm42688.data.acc_z);     tft180_show_string(0, 5 * UI_ROW_H, ui_buf);
+    sprintf(ui_buf, "Acc_X: %8.2f   ", imu_icm42688.data.acc_x);     tft180_show_string(0, 6 * UI_ROW_H, ui_buf);
+    sprintf(ui_buf, "Acc_Y: %8.2f   ", imu_icm42688.data.acc_y);     tft180_show_string(0, 7 * UI_ROW_H, ui_buf);
+    sprintf(ui_buf, "Gyr_Z: %8.2f   ", imu_icm42688.data.gyro_z);     tft180_show_string(0, 8 * UI_ROW_H, ui_buf);
+    sprintf(ui_buf, "Gyr_X: %8.2f   ", imu_icm42688.data.gyro_x);     tft180_show_string(0, 9 * UI_ROW_H, ui_buf);
+    sprintf(ui_buf, "Gyr_Y: %8.2f   ", imu_icm42688.data.gyro_y);     tft180_show_string(0, 10 * UI_ROW_H, ui_buf);
+
+    tft180_show_string(0, 11 * UI_ROW_H, "Spd LF: ");     tft180_show_float(10 * UI_COL_W, 11 * UI_ROW_H, wheels.lf, 3, 1);
+    tft180_show_string(0, 12 * UI_ROW_H, "Spd LB: ");     tft180_show_float(10 * UI_COL_W, 12 * UI_ROW_H, wheels.lb, 3, 1);
+    tft180_show_string(0, 13 * UI_ROW_H, "Spd RF: ");    tft180_show_float(10 * UI_COL_W, 13 * UI_ROW_H, wheels.rf, 3, 1);
+    tft180_show_string(0, 14 * UI_ROW_H, "Spd RB: ");    tft180_show_float(10 * UI_COL_W, 14 * UI_ROW_H, wheels.rb, 3, 1);
 }
 
 // 缁樺埗鍙傛暟璋冭妭椤甸潰
@@ -651,17 +659,13 @@ void draw_float_item(uint8_t row, const char* name, float val, bool is_selected,
     
     // 娓叉煋缂栬緫鏍囪瘑锛堜笌鏁板€煎尯淇濇寔涓嶉噸鍙狅級
     if (is_selected && is_editing_this) {
-        tft180_show_string(9 * UI_COL_W, row * UI_ROW_H, "[E]");
+        tft180_show_string(8 * UI_COL_W, row * UI_ROW_H, "[E]");
     } else {
-        tft180_show_string(9 * UI_COL_W, row * UI_ROW_H, "   ");
+        tft180_show_string(8 * UI_COL_W, row * UI_ROW_H, "   ");
     }
 
-    // 鍥哄畾浠庣 12 鍒楀紑濮嬫樉绀猴紝杈冨師鍏堝乏绉讳竴鍒楋紝閬垮厤澧炲姞浣嶆暟鍚庤秺鐣屻€�
-    tft180_show_float(12 * UI_COL_W,
-                    row * UI_ROW_H,
-                    val,
-                    5,
-                    2);
+    // 鏄剧ず 3 浣嶅皬鏁版椂锛屾暣浣撳乏绉讳竴鍒楋紝閬垮厤鏈€鍙充晶瓒婄晫銆�
+    tft180_show_float(11 * UI_COL_W, row * UI_ROW_H, val, 5, 3);
 }
 
 // ================= 鍩虹缁樺浘杈呭姪鍑芥暟 ===================
diff --git a/project/mdk/rt1064.uvoptx b/project/mdk/rt1064.uvoptx
index 3b5b795..b648df1 100644
--- a/project/mdk/rt1064.uvoptx
+++ b/project/mdk/rt1064.uvoptx
@@ -409,12 +409,12 @@
     <File>
       <GroupNumber>1</GroupNumber>
       <FileNumber>4</FileNumber>
-      <FileType>5</FileType>
+      <FileType>8</FileType>
       <tvExp>0</tvExp>
       <tvExpOptDlg>0</tvExpOptDlg>
       <bDave2>0</bDave2>
-      <PathWithFileName>..\App\GameManage.h</PathWithFileName>
-      <FilenameWithoutPath>GameManage.h</FilenameWithoutPath>
+      <PathWithFileName>..\App\GameManageDemo.cpp</PathWithFileName>
+      <FilenameWithoutPath>GameManageDemo.cpp</FilenameWithoutPath>
       <RteFlg>0</RteFlg>
       <bShared>0</bShared>
     </File>
@@ -425,8 +425,8 @@
       <tvExp>0</tvExp>
       <tvExpOptDlg>0</tvExpOptDlg>
       <bDave2>0</bDave2>
-      <PathWithFileName>..\App\TestMap.cpp</PathWithFileName>
-      <FilenameWithoutPath>TestMap.cpp</FilenameWithoutPath>
+      <PathWithFileName>..\App\GameManageMock.cpp</PathWithFileName>
+      <FilenameWithoutPath>GameManageMock.cpp</FilenameWithoutPath>
       <RteFlg>0</RteFlg>
       <bShared>0</bShared>
     </File>
@@ -437,8 +437,8 @@
       <tvExp>0</tvExp>
       <tvExpOptDlg>0</tvExpOptDlg>
       <bDave2>0</bDave2>
-      <PathWithFileName>..\App\TestMap.h</PathWithFileName>
-      <FilenameWithoutPath>TestMap.h</FilenameWithoutPath>
+      <PathWithFileName>..\App\GameManage.h</PathWithFileName>
+      <FilenameWithoutPath>GameManage.h</FilenameWithoutPath>
       <RteFlg>0</RteFlg>
       <bShared>0</bShared>
     </File>    draw_item(2, "Dashboard",  ctx.cursor_idx == 0);
    draw_item(3, "Odometry",   ctx.cursor_idx == 1);
    draw_item(4, "Tuning",     ctx.cursor_idx == 2);
    draw_item(5, "Save Config",ctx.cursor_idx == 3);
    draw_item(6, "Load Config",ctx.cursor_idx == 4);
    draw_item(7, "Close Menu", ctx.cursor_idx == 5); 
    draw_item(8, "forward",ctx.cursor_idx == 6);
    draw_item(9, "right", ctx.cursor_idx == 7); 
}

// 绘制模式选择页面
void draw_mode_select() {
    tft180_show_string(0, 0, "-- SELECT MODE --");
    
    draw_item(2, "1. Demo (Virtual)", ctx.cursor_idx == 0);
    draw_item(3, "2. Mock (Run Car) ", ctx.cursor_idx == 1);

    tft180_show_string(0, 7 * UI_ROW_H, "Tips:");
    if (ctx.cursor_idx == 1) {
        tft180_show_string(0, 8 * UI_ROW_H, "> Motor: ENABLE");
        tft180_show_string(0, 9 * UI_ROW_H, "> ART1 : MOCKED");
    } else {
        tft180_show_string(0, 8 * UI_ROW_H, "> Motor: DISABLE");
        tft180_show_string(0, 9 * UI_ROW_H, "> View : ANIMATION");
    }
}

// 绘制地图选择页面
void draw_map_select() {
    tft180_show_string(0, 0, "-- SELECT MAP --");
    
    uint8_t count = App::GameEngine::get_mock_map_count();

    // 进度提示 (例如 1/3)，放在右上角
    tft180_show_int(14 * UI_COL_W, 0, ctx.map_cursor_idx + 1, 2);
    tft180_show_string(16 * UI_COL_W, 0, "/");
    tft180_show_int(17 * UI_COL_W, 0, count, 2);
    
    // 利用已有的 PARAMS_PER_PAGE 行数限制渲染滚动列表
    for (int i = 0; i < PARAMS_PER_PAGE; i++) {
        int item_idx = ctx.map_scroll_offset + i;
        if (item_idx >= count) {
            // 清理多余行，打印 21 个空格覆盖一整行
            tft180_show_string(0, (i + 1) * UI_ROW_H, "                     ");
            continue;
        }

        draw_item(i + 1, App::GameEngine::get_mock_map_name(item_idx), ctx.map_cursor_idx == item_idx);
    }
}

// 里程计和硬件监控页面
void draw_odometry_data() {
    tft180_show_string(0, 0, "-- ODO & HW --");
    auto pos = App::g_state.physical.pose;
    auto wheels = App::g_state.physical.current_wheel_speed;

    tft180_show_string(0, 2 * UI_ROW_H, "Global X: ");   tft180_show_float(10 * UI_COL_W, 2 * UI_ROW_H, pos.x, 3, 1);
    tft180_show_string(0, 3 * UI_ROW_H, "Global Y: ");   tft180_show_float(10 * UI_COL_W, 3 * UI_ROW_H, pos.y, 3, 1);
    char ui_buf[32]; 
    sprintf(ui_buf, "Yaw: %8.2f   ", pos.yaw);     tft180_show_string(0, 4 * UI_ROW_H, ui_buf);
    sprintf(ui_buf, "Acc_Z: %8.2f   ", imu_icm42688.data.acc_z);     tft180_show_string(0, 5 * UI_ROW_H, ui_buf);
    sprintf(ui_buf, "Acc_X: %8.2f   ", imu_icm42688.data.acc_x);     tft180_show_string(0, 6 * UI_ROW_H, ui_buf);
    sprintf(ui_buf, "Acc_Y: %8.2f   ", imu_icm42688.data.acc_y);     tft180_show_string(0, 7 * UI_ROW_H, ui_buf);
    sprintf(ui_buf, "Gyr_Z: %8.2f   ", imu_icm42688.data.gyro_z);     tft180_show_string(0, 8 * UI_ROW_H, ui_buf);
    sprintf(ui_buf, "Gyr_X: %8.2f   ", imu_icm42688.data.gyro_x);     tft180_show_string(0, 9 * UI_ROW_H, ui_buf);
    sprintf(ui_buf, "Gyr_Y: %8.2f   ", imu_icm42688.data.gyro_y);     tft180_show_string(0, 10 * UI_ROW_H, ui_buf);

    tft180_show_string(0, 11 * UI_ROW_H, "Spd LF: ");     tft180_show_float(10 * UI_COL_W, 11 * UI_ROW_H, wheels.lf, 3, 1);
    tft180_show_string(0, 12 * UI_ROW_H, "Spd LB: ");     tft180_show_float(10 * UI_COL_W, 12 * UI_ROW_H, wheels.lb, 3, 1);
    tft180_show_string(0, 13 * UI_ROW_H, "Spd RF: ");    tft180_show_float(10 * UI_COL_W, 13 * UI_ROW_H, wheels.rf, 3, 1);
    tft180_show_string(0, 14 * UI_ROW_H, "Spd RB: ");    tft180_show_float(10 * UI_COL_W, 14 * UI_ROW_H, wheels.rb, 3, 1);
}

// 绘制参数调节页面
void draw_tune_params() {
    tft180_show_string(0, 0, "PARAMETERS");
    
    // 进度提示 (例如 1/8)，放在右上角
    tft180_show_int(14 * UI_COL_W, 0, ctx.cursor_idx + 1, 2);
    tft180_show_string(16 * UI_COL_W, 0, "/");
    tft180_show_int(17 * UI_COL_W, 0, DICT_SIZE, 2);
    
    for (int i = 0; i < PARAMS_PER_PAGE; i++) {
        int item_idx = ctx.scroll_offset + i;
        if (item_idx >= DICT_SIZE) {
            // 清理多余行，打印 21 个空格正好覆盖一整行
            tft180_show_string(0, (i + 1) * UI_ROW_H, "                     ");
            continue;
        }

        draw_float_item(i + 1, 
            tune_dict[item_idx].name, 
            *(tune_dict[item_idx].val_ptr), 
            ctx.cursor_idx == item_idx, 
            ctx.is_editing && (ctx.cursor_idx == item_idx));
    }
}

// 仪表盘页面：根据游戏状态动态显示信息
void draw_dashboard() {
    App::GameEngine::RenderContext render_ctx = App::GameEngine::get_render_context();
    auto& game = App::g_state.game;

    // 1. 局部组装字符串
    char hud_line0[22] = {0}, hud_line1[22] = {0}, hud_line2[22] = {0};
    snprintf(hud_line1, sizeof(hud_line1), "Stage: %d", game.is_advanced_stage ? 2 : 1);

    if (game.is_demo_mode) {
        switch(game.phase) {
            case GamePhase::NONE:                  snprintf(hud_line0, 22, "Phase: NONE       "); break;
            case GamePhase::WAIT_FOR_VISION:       snprintf(hud_line0, 22, "Phase: WAITING MAP"); break;
            case GamePhase::PLAN_PATROL:           snprintf(hud_line0, 22, "Phase: PLAN PATROL"); break;
            case GamePhase::ANIMATE_PATROL_DEMO:   snprintf(hud_line0, 22, "Phase: DEMO PATROL"); break;
            case GamePhase::BIND_SEMANTICS:        snprintf(hud_line0, 22, "Phase: BINDING    "); break;
            case GamePhase::PLAN_SOKOBAN:          snprintf(hud_line0, 22, "Phase: PLAN SOKO  "); break;
            case GamePhase::ANIMATE_DEMO:          snprintf(hud_line0, 22, "Phase: DEMO PUSH  "); break;
            case GamePhase::PLAN_RETURN_HOME:      snprintf(hud_line0, 22, "Phase: PLAN RTN   "); break;
            case GamePhase::ANIMATE_RETURN_DEMO:   snprintf(hud_line0, 22, "Phase: DEMO RTN   "); break;
            case GamePhase::FINISHED:              snprintf(hud_line0, 22, "Phase: FINISHED   "); break;
            case GamePhase::ERROR_OCCURRED:        snprintf(hud_line0, 22, "Phase: ERROR      "); break;
            default:                               snprintf(hud_line0, 22, "Phase: Other     "); break;
        }
        if (game.phase == GamePhase::ERROR_OCCURRED) {
            tft180_show_int (14 * UI_COL_W, 0, App::g_state.game.error_stage, 2);
        }
        if (game.phase == GamePhase::ANIMATE_PATROL_DEMO || game.phase == GamePhase::BIND_SEMANTICS || game.phase == GamePhase::PLAN_SOKOBAN) {
            snprintf(hud_line2, 22, "Bm:%3dms GT:%3dms", (int)render_ctx.bomb_plan_time_ms, (int)render_ctx.patrol_plan_time_ms);
        } else if (game.phase == GamePhase::ANIMATE_DEMO || game.phase == GamePhase::FINISHED) {
            snprintf(hud_line2, 22, "IDA* Time: %4dms", (int)render_ctx.push_plan_time_ms);
        } else {
            snprintf(hud_line2, 22, "Plan Time: --  ms");
        }
    } else {
        switch(game.phase) {
            case GamePhase::NONE:                  snprintf(hud_line0, 22, "P: NONE      "); break;
            case GamePhase::INIT_CALIBRATE:        snprintf(hud_line0, 22, "P: INIT      "); break;
            case GamePhase::EXIT_START_ZONE:       snprintf(hud_line0, 22, "P: EXIT      "); break;
            case GamePhase::WAIT_FOR_VISION:       snprintf(hud_line0, 22, "P: WAIT ART1 "); break;
            case GamePhase::EXEC_ACTION_DISPATCH:  snprintf(hud_line0, 22, "P: ACT DISP  "); break;
            case GamePhase::EXEC_PATROL_MOVE:      snprintf(hud_line0, 22, "P: EXEC WATCH"); break;
            case GamePhase::EXEC_ALIGN_YAW:        snprintf(hud_line0, 22, "P: EXEC YAW  "); break;
            case GamePhase::WAIT_ART2_CAPTURE_ACK: snprintf(hud_line0, 22, "P: WAIT ART2 "); break;
            case GamePhase::EXEC_BOMB_PUSH:        snprintf(hud_line0, 22, "P: EXEC BOMB "); break;
            case GamePhase::EXEC_SOKOBAN:          snprintf(hud_line0, 22, "P: EXEC BOX  "); break;
            case GamePhase::EXEC_RETURN_HOME:      snprintf(hud_line0, 22, "P: EXEC HOME "); break;
            case GamePhase::FINISHED:              snprintf(hud_line0, 22, "P: FINISHED  "); break;
            case GamePhase::ERROR_OCCURRED:        snprintf(hud_line0, 22, "P: ERROR : %d", App::g_state.game.error_stage); break;
            default:                               snprintf(hud_line0, 22, "P: COMPUTING "); break;
        }

        snprintf(hud_line2, 22, "Plan Time: --  ms");
    }

    // 2. 顶部 HUD 防闪烁渲染
    static char last_hud0[22] = {0}, last_hud1[22] = {0}, last_hud2[22] = {0};
    
    // 如果外部触发了强制重绘，顺便把文字的记忆清空
    if (App::g_state.debug.need_bg_redraw) {
        last_hud0[0] = '\0'; last_hud1[0] = '\0'; last_hud2[0] = '\0';
    }

    // last_hud 与 hud_line不一样，才用空格覆盖旧的，然后画上新字
    if (strncmp(last_hud0, hud_line0, 22) != 0) {
        tft180_show_string(0, 0, "                     "); 
        tft180_show_string(0, 0, hud_line0);
        strncpy(last_hud0, hud_line0, 22);
    }
    if (strncmp(last_hud1, hud_line1, 22) != 0) {
        tft180_show_string(0, 1 * UI_ROW_H, "                     ");
        tft180_show_string(0, 1 * UI_ROW_H, hud_line1);
        strncpy(last_hud1, hud_line1, 22);
    }
    if (strncmp(last_hud2, hud_line2, 22) != 0) {
        tft180_show_string(0, 2 * UI_ROW_H, "                     ");
        tft180_show_string(0, 2 * UI_ROW_H, hud_line2);
        strncpy(last_hud2, hud_line2, 22);
    }

    // 3. 在内存中合成静态画布 （地图+目标+箱子+炸弹+路径+观测点）
    uint8_t canvas[16][12] = {0};
    if (render_ctx.map) {
        for(int y = 0; y < MAP_MAX_HEIGHT; y++) {
            for(int x = 0; x < MAP_MAX_WIDTH; x++) {
                int8_t tile = (*render_ctx.map)[y][x];
                if      (tile == 1) canvas[y][x] |= TL_WALL;
                else if (tile == 2) canvas[y][x] |= TL_BOX;
                else if (tile == 3) canvas[y][x] |= TL_TGT;
                else if (tile == 4) canvas[y][x] |= TL_BOMB;
            }
        }
    }
    
    if (render_ctx.path_ptr) {
        for(size_t i = render_ctx.path_start_idx; i < render_ctx.path_ptr->size(); i++) 
            canvas[(*render_ctx.path_ptr)[i].y][(*render_ctx.path_ptr)[i].x] |= TL_PATH;
    }
    if (render_ctx.actions_ptr) {
        for(size_t i = render_ctx.action_start_idx; i < render_ctx.actions_ptr->size(); i++) 
            if(!(*render_ctx.actions_ptr)[i].is_bomb_task) canvas[(*render_ctx.actions_ptr)[i].obs.pos.y][(*render_ctx.actions_ptr)[i].obs.pos.x] |= TL_CRS;
    }

    // 像素级小车独立计算系统
    int map_start_y = 3 * UI_ROW_H + 4;
    static float last_car_sx = -1.0f, last_car_sy = -1.0f;
    float current_car_sx = 0.0f, current_car_sy = 0.0f;
    
    // 只有在视觉模块完成，进入寻路阶段后才开始绘制小车
    bool should_draw_car = (game.phase > GamePhase::WAIT_FOR_VISION);

    if (should_draw_car) {
        if (game.is_demo_mode) {
            // 动画模式下，直接按网格坐标投射到屏幕像素 (8个像素一格)
            current_car_sx = render_ctx.player_pos.y * 8.0f;
            current_car_sy = render_ctx.player_pos.x * 8.0f + map_start_y;
        } else {
            auto pos = App::g_state.physical.pose;
            // X轴物理坐标对应屏幕上的 Y方向 (sx)，Y轴对应 X方向 (sy)
            // 物理网格 1格 = 20cm，屏幕上 1格 = 8像素。缩放系数为 8/20 = 0.4
            current_car_sx = (pos.y - SystemConfig::MAP_OFFSET_Y) * 0.4f;
            current_car_sy = (pos.x - SystemConfig::MAP_OFFSET_X) * 0.4f + map_start_y;
            
            // 安全限幅防越界
            if (current_car_sx < 0) current_car_sx = 0; else if (current_car_sx > 15*8) current_car_sx = 15*8;
            if (current_car_sy < map_start_y) current_car_sy = map_start_y; else if (current_car_sy > map_start_y + 11*8) current_car_sy = map_start_y + 11*8;
        }

        // 把小车压过的背景缓存设为 0xFF，触发这几个格子的自动重绘
        if (last_car_sx >= 0.0f) {
            fill_rect((int)last_car_sx + 2, (int)last_car_sy + 2, 4, 4, RGB565_WHITE);
            
            int old_gy = (int)last_car_sx / 8;
            int old_gx = (int)(last_car_sy - map_start_y) / 8;
            for(int dy = 0; dy <= 1; dy++) {
                for(int dx = 0; dx <= 1; dx++) {
                    if (old_gy + dy < 16 && old_gx + dx < 12) {
                        ctx.back_buffer[old_gy + dy][old_gx + dx] = 0xFF; // 强行设为无效，触发重绘
                    }
                }
            }
        }
    }

    // 4. O(1) 脏矩形底图增量渲染 
    if (App::g_state.debug.need_bg_redraw) { 
        memset(ctx.back_buffer, 0xFF, sizeof(ctx.back_buffer)); // 失效显存，强制全刷
        
        // 静态背景上的附加涂装：彩色炸弹框
        if (render_ctx.bomb_tasks_ptr) {
            uint16_t b_colors[] = {RGB565_RED, RGB565_BLUE, RGB565_CYAN, RGB565_MAGENTA}; 
            for (int i = 0; i < render_ctx.bomb_tasks_ptr->size(); ++i) {
                uint16_t color = b_colors[i % 4];
                point bs = (*render_ctx.bomb_tasks_ptr)[i].bomb_start;
                point tw = (*render_ctx.bomb_tasks_ptr)[i].target_wall;

                int bs_sx = bs.y * 8, bs_sy = map_start_y + bs.x * 8;
                tft180_draw_line(bs_sx, bs_sy, bs_sx + 7, bs_sy, color);
                tft180_draw_line(bs_sx, bs_sy + 7, bs_sx + 7, bs_sy + 7, color);
                tft180_draw_line(bs_sx, bs_sy, bs_sx, bs_sy + 7, color);
                tft180_draw_line(bs_sx + 7, bs_sy, bs_sx + 7, bs_sy + 7, color);

                if ((*render_ctx.map)[tw.y][tw.x] == 1) { // 墙还没被炸毁的话，画框
                    int tw_sx = tw.y * 8, tw_sy = map_start_y + tw.x * 8;
                    tft180_draw_line(tw_sx, tw_sy, tw_sx + 7, tw_sy, color);
                    tft180_draw_line(tw_sx, tw_sy + 7, tw_sx + 7, tw_sy + 7, color);
                    tft180_draw_line(tw_sx, tw_sy, tw_sx, tw_sy + 7, color);
                    tft180_draw_line(tw_sx + 7, tw_sy, tw_sx + 7, tw_sy + 7, color);
                }
            }
        }
        App::g_state.debug.need_bg_redraw = false; 
    }

    // 核心底图渲染循环
    for(int y=0; y<16; y++) {
        for(int x=0; x<12; x++) {
            // 只有画布状态与显存不一致时才重绘这个格子，达到增量更新的效果
            if (canvas[y][x] != ctx.back_buffer[y][x]) {  
                int sx = y * 8, sy = x * 8 + map_start_y;
                
                fill_rect(sx + 1, sy + 1, 7, 7, (canvas[y][x] & TL_WALL) ? RGB565_GRAY : RGB565_WHITE);
                
                if (canvas[y][x] & TL_TGT)  fill_rect(sx + 1, sy + 1, 6, 6, RGB565_PURPLE);
                if (canvas[y][x] & TL_BOX)  fill_rect(sx + 1, sy + 1, 6, 6, RGB565_YELLOW);
                if (canvas[y][x] & TL_BOMB) { fill_rect(sx + 1, sy + 1, 6, 6, RGB565_BLACK); fill_rect(sx + 3, sy + 3, 2, 2, RGB565_RED); }

                if (canvas[y][x] & TL_PATH) fill_rect(sx + 3, sy + 3, 2, 2, RGB565_BLUE);
                if (canvas[y][x] & TL_CRS)  { tft180_draw_line(sx+2, sy+2, sx+6, sy+6, RGB565_BLUE); tft180_draw_line(sx+2, sy+6, sx+6, sy+2, RGB565_BLUE); }
                ctx.back_buffer[y][x] = canvas[y][x]; 
            }
        }
    }

    // 5. 最后在底图之上，绘制自由滑行的小车
    if (should_draw_car) {
        // float 转 int，+2 是为了让 4x4 的小车正好居中在一个 8x8 的格子里
        fill_rect((int)current_car_sx + 2, (int)current_car_sy + 2, 4, 4, RGB565_GREEN);
        
        // 记录历史位置，用于下一帧擦除
        last_car_sx = current_car_sx;
        last_car_sy = current_car_sy;
    } else {
        last_car_sx = -1.0f; // 重置小车状态
    }
}

// ================= 基础绘图辅助函数 ===================

void draw_item(uint8_t row, const char* name, bool is_selected) {
    if (is_selected) tft180_show_string(0, row * UI_ROW_H, ">"); 
    else tft180_show_string(0, row * UI_ROW_H, " "); 
    
    char buf[22];
    snprintf(buf, sizeof(buf), "%-20s", name); 
    tft180_show_string(1 * UI_COL_W, row * UI_ROW_H, buf);
}

void draw_float_item(uint8_t row, const char* name, float val, bool is_selected, bool is_editing_this) {
    // 渲染光标和名称 (占用 0 ~ 9 列)
    draw_item(row, name, is_selected);
    
    // 渲染编辑标识（与数值区保持不重叠）
    if (is_selected && is_editing_this) {
        tft180_show_string(8 * UI_COL_W, row * UI_ROW_H, "[E]");
    } else {
        tft180_show_string(8 * UI_COL_W, row * UI_ROW_H, "   ");
    }

    // 显示 3 位小数时，整体左移一列，避免最右侧越界。
    tft180_show_float(11 * UI_COL_W, row * UI_ROW_H, val, 5, 3);
}

// ================= 基础绘图辅助函数 ===================

void fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color) {
    for (uint8_t i = 0; i < h; ++i) {
        tft180_draw_line(x, y + i, x + w - 1, y + i, color);
    }
}

} // namespace Subsystem::Display