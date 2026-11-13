#include <M5Unified.h>
#include <Ticker.h> // ESP32標準のタイマー割込み用ライブラリ
#include "driver/twai.h" // ESP32標準のCAN(TWAI)ドライバをインクルード

// -------------------------------------------------------------
// CAN通信（TWAI）ピンと速度の設定
// -------------------------------------------------------------
#define CAN_TX_PIN   GPIO_NUM_5  // 使用する環境のTXピンに合わせてください (例: G5)
#define CAN_RX_PIN   GPIO_NUM_6  // 使用する環境のRXピンに合わせてください (例: G6)
#define CAN_BAUDRATE TWAI_TIMING_CONFIG_500KBITS() // 通信速度: 500kbps (1MBITSなどへ変更可能)

// タイマー管理用のオブジェクト
Ticker timer;

// タイマー割込みが発生したことをloopに伝える「フラグ」
// ※割込み内で書き換える変数は必ず「volatile」を付けます
volatile bool timerFlag = false;

// タイマー割込みが発生したときに実行される関数（ISR: 割込みサービスルーチン）
// ※この中ではSerial.printやM5.Displayなどの重い処理は絶対にNGです
void onTimer() {
  timerFlag = true; // loop側に「時間になったよ」と知らせるフラ取っ手（フラグ）を立てる
}

// -------------------------------------------------------------
// CANの初期化関数
// -------------------------------------------------------------
void initCAN() {
  // 一般設定（ピン、モード、キューサイズの設定）
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  
  // 速度設定
  twai_timing_config_t t_config = CAN_BAUDRATE;
  
  // フィルター設定（すべてのIDを受け入れる）
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // ドライバのインストールと起動
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("CAN driver installed successfully.");
  } else {
    Serial.println("Failed to install CAN driver.");
  }

  if (twai_start() != ESP_OK) {
    Serial.println("Failed to start CAN driver.");
  }
}

void setup(void) {
  // M5Stackの初期化
  auto cfg = M5.config();
  M5.begin(cfg);

  // USBシリアル通信の開始（USB CDC On Boot を Enabled に設定してください）
  Serial.begin(115200);
  
  // 互換APIの初期化（getAhrsData等を使用可能にするため）
  M5.Imu.init(); 

  // 画面の初期設定
  M5.Display.setTextSize(1.5);

  // CAN通信の初期化
  initCAN();

  // タイマー割込みの設定（0.01秒 = 10ミリ秒 ごとに onTimer 関数を実行）
  // 引数：(秒数, 実行する関数名)
  timer.attach(0.01, onTimer); 

}

void loop(void) {
  // タイマー割込みによってフラグが true になったら処理を実行
  if (timerFlag) {
    timerFlag = false; // フラグを元に戻す

  // センサーの内部状態と姿勢計算を更新
     M5.Imu.update();

  // 加速度とジャイロの生データを取得
     auto imuData = M5.Imu.getImuData();

  // 姿勢角（ピッチ、ロール、ヨー）を互換関数から取得
     float pitch = 0.0f;
     float roll  = 0.0f;
     float yaw   = 0.0f;
      M5.Imu.getGyroData(&pitch, &roll, &yaw);

    // -------------------------------------------------------------
    // 【追加】CAN送信データの作成と送信処理
    // -------------------------------------------------------------
    // float（4バイト）の値をそのまま送るため、100倍して「整数（int16_t）」に変換
    int16_t can_roll  = (int16_t)(roll * 100.0f);
    int16_t can_pitch = (int16_t)(pitch * 100.0f);
    int16_t can_yaw   = (int16_t)(yaw * 100.0f);

    twai_message_t tx_msg;
    tx_msg.identifier = 0x787;         // CAN ID (お好みのIDに変更可能：11bit標準ID)
    tx_msg.extd = 0;                   // 標準フレーム (拡張フレームの場合は1)
    tx_msg.rtr = 0;                    // データフレーム
    tx_msg.data_length_code = 6;       // 送信データ長 (2バイト×3項目 = 6バイト)

    // バイト配列にデータを分解して格納 (ビッグエンディアン形式)
    tx_msg.data[0] = (can_roll >> 8) & 0xFF;
    tx_msg.data[1] = can_roll & 0xFF;
    tx_msg.data[2] = (can_pitch >> 8) & 0xFF;
    tx_msg.data[3] = can_pitch & 0xFF;
    tx_msg.data[4] = (can_yaw >> 8) & 0xFF;
    tx_msg.data[5] = can_yaw & 0xFF;

    // CANバスへ送信
    if (twai_transmit(&tx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
      // 送信成功時、画面の下部にドットを表示して確認できるようにする
      M5.Display.fillCircle(120, 120, 3, GREEN);
    } else {
      M5.Display.fillCircle(120, 120, 3, RED); // 送信失敗（バスが浮いている等）
    }
  // シリアルプロッタ等で見やすいように「タグ:値」のカンマ区切りで出力
  //   Serial.printf("AccX:%.2f,AccY:%.2f,AccZ:%.2f,", imuData.accel.x, imuData.accel.y, imuData.accel.z);
  //   Serial.printf("GyroX:%.2f,GyroY:%.2f,GyroZ:%.2f,", imuData.gyro.x, imuData.gyro.y, imuData.gyro.z);
  //   Serial.printf("Roll:%.2f,Pitch:%.2f,Yaw:%.2f\n", roll, pitch, yaw);

  // 画面（LCD）側にもリアルタイムに描画
     M5.Display.startWrite();
     M5.Display.setCursor(0, 0);
  
     M5.Display.printf("[Accel]\nX:%.2f Y:%.2f Z:%.2f\n\n", imuData.accel.x, imuData.accel.y, imuData.accel.z);
     M5.Display.printf("[Gyro]\nX:%.1f Y:%.1f Z:%.1f\n\n", imuData.gyro.x, imuData.gyro.y, imuData.gyro.z);
     M5.Display.printf("[Pose]\nR:%.1f P:%.1f Y:%.1f\n", roll, pitch, yaw);
  
     M5.Display.endWrite();
  
      // float（4バイト）の値をそのまま送るため、100倍して「整数（int16_t）」に変換
    int16_t can_ax  = (int16_t)(imuData.accel.x * 100.0f);
    int16_t can_ay  = (int16_t)(imuData.accel.y * 100.0f);
    int16_t can_az  = (int16_t)(imuData.accel.z * 100.0f);
  
  
    tx_msg.identifier = 0x788;         // CAN ID (お好みのIDに変更可能：11bit標準ID)
    tx_msg.extd = 0;                   // 標準フレーム (拡張フレームの場合は1)
    tx_msg.rtr = 0;                    // データフレーム
    tx_msg.data_length_code = 6;       // 送信データ長 (2バイト×3項目 = 6バイト)

    // バイト配列にデータを分解して格納 (ビッグエンディアン形式)
    tx_msg.data[0] = (can_ax >> 8) & 0xFF;
    tx_msg.data[1] = can_ax & 0xFF;
    tx_msg.data[2] = (can_ay >> 8) & 0xFF;
    tx_msg.data[3] = can_ay & 0xFF;
    tx_msg.data[4] = (can_az >> 8) & 0xFF;
    tx_msg.data[5] = can_az & 0xFF;

    // CANバスへ送信
    if (twai_transmit(&tx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
      // 送信成功時、画面の下部にドットを表示して確認できるようにする
      M5.Display.fillCircle(120, 120, 3, GREEN);
    } else {
      M5.Display.fillCircle(120, 120, 3, RED); // 送信失敗（バスが浮いている等）
    }
  // シリアルプロッタ等で見やすいように「タグ:値」のカンマ区切りで出力
  //   Serial.printf("AccX:%.2f,AccY:%.2f,AccZ:%.2f,", imuData.accel.x, imuData.accel.y, imuData.accel.z);
  //   Serial.printf("GyroX:%.2f,GyroY:%.2f,GyroZ:%.2f,", imuData.gyro.x, imuData.gyro.y, imuData.gyro.z);
  //   Serial.printf("Roll:%.2f,Pitch:%.2f,Yaw:%.2f\n", roll, pitch, yaw);
  }
}
