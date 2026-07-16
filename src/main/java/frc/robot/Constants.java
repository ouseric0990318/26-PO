package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    // 🟢 放在類別最外層，讓所有舊檔案（不管有沒有寫 DriveConstants.）都能無腦抓到這兩個變數
    public static final double wheelRadius = Units.inchesToMeters(2.0);
    public static final double driveGearRatio = 6.03;

    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        
        // 🟢 補上副控 (Operator) 的搖桿 Port，通常設定為 1
        public static final int kOperatorControllerPort = 1; 
        
        // 🟢 補上搖桿死區 (Deadband)，通常設定在 0.05 到 0.1 之間
        public static final double kDeadband = 0.1; 
    }

    public static final class DriveConstants {
        // ==================== CANivore 通訊設定 ====================
        // Tuner X 中給 CANivore 取的網卡名稱
        public static final String kCANbusName = "base1"; 

        // ==================== 陀螺儀 (Gyro) ID ====================
        public static final int kPigeon2Id = 13;

        // ==================== 4 個輪子的物理與機械參數 ====================
        // 1. 舊代碼使用的輪胎半徑 (以標準 4 吋輪為例，半徑為 2 吋，並轉換成公尺)
        public static final double wheelRadius = Units.inchesToMeters(2.0); 
    
        // 2. 舊代碼使用的驅動齒輪比 (SDS MK4i L3 為 5.36，L2 為 6.75)
        public static final double driveGearRatio = 6.03; 

        // 為了相容新舊程式碼，我們可以讓新變數直接對接舊變數：
        public static final double kWheelDiameterMeters = wheelRadius * 2.0;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kDriveGearRatio = driveGearRatio; 
        public static final double kSteerGearRatio = 150.0 / 7.0; 
    
        // 物理轉換：將馬達旋轉圈數 (Rotations) 轉換成輪胎實際行進公尺 (Meters)
        public static final double kDriveRotationsToMeters = kWheelCircumferenceMeters / kDriveGearRatio;

        // ==================== 4 組模組 CAN ID 與 絕對編碼器偏移量 ====================
        /*
         * 💡 註：Offsets 的數值是你在實體車上將輪子手動對齊正前方後，
         * 從 Tuner X 讀取到的 CANcoder 絕對角度值（單位：轉/Rotations）。
         */
        
        // 1. 左前模組 (Front Left - Index 0)
        public static final int kFLDriveId = 2;
        public static final int kFLSteerId = 1;
        public static final int kFLEncoderId = 9;
        public static final double kFLOffset = 0.497559; // 填入實際量測的 Offset

        // 2. 右前模組 (Front Right - Index 1)
        public static final int kFRDriveId = 4;
        public static final int kFRSteerId = 3;
        public static final int kFREncoderId = 10;
        public static final double kFROffset = -0.498535;

        // 3. 左後模組 (Back Left - Index 2)
        public static final int kBLDriveId = 6;
        public static final int kBLSteerId = 5;
        public static final int kBLEncoderId = 11;
        public static final double kBLOffset = -0.498779;

        // 4. 右後模組 (Back Right - Index 3)
        public static final int kBRDriveId = 8;
        public static final int kBRSteerId = 7;
        public static final int kBREncoderId = 12;
        public static final double kBROffset = -0.499512;

        // ==================== 運動學配置 (Kinematics) ====================
        // 假設車身尺寸為 24 吋 x 24 吋 (左右輪距與前後軸距)
        public static final double kTrackWidthX = Units.inchesToMeters(20.75); 
        public static final double kTrackWidthY = Units.inchesToMeters(22.5); 

        public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            new Translation2d(kTrackWidthX / 2.0, kTrackWidthY / 2.0),   // 左前 (FL)
            new Translation2d(kTrackWidthX / 2.0, -kTrackWidthY / 2.0),  // 右前 (FR)
            new Translation2d(-kTrackWidthX / 2.0, kTrackWidthY / 2.0),  // 左後 (BL)
            new Translation2d(-kTrackWidthX / 2.0, -kTrackWidthY / 2.0) // 右後 (BR)
        );

        // ==================== 電流與安全限制 (Current Limits) ====================
        // 驅動馬達 (Kraken X60) 安全設定 (防止大電流燒毀，並能抑止起步打滑)
        public static final double kDriveCurrentLimit = 60.0;        // 續航電流限制 (Amps)
        public static final double kDriveStatorCurrentLimit = 80.0; // 最大定子電流限制

        // 轉向馬達 (Talon FX) 安全設定
        public static final double kSteerCurrentLimit = 40.0;
        public static final double kSteerStatorCurrentLimit = 80.0;

        // ==================== 馬達閉迴路 PID 參數 (Closed-Loop Gains) ====================
        // 驅動馬達速度控制 PID & 前饋 (Slot 0)
        public static final double kDriveKP = 0.12;
        public static final double kDriveKI = 0.0;
        public static final double kDriveKD = 0.0;
        public static final double kDriveKS = 0.15; // 克服靜摩擦力電壓 (Volts)
        public static final double kDriveKV = 0.12; // 速度前饋 (Volts per rps)

        // 轉向馬達位置控制 PID (Slot 0 - 搭配 FusedCANCoder)
        //  因為 FusedCANCoder 計算單位為「轉 (Rotations)」，所以 P 值會比較大
        public static final double kSteerKP = 50.0; 
        public static final double kSteerKI = 0.0;
        public static final double kSteerKD = 0.2;

        // 🟢 補上這三行，徹底解決剩餘的編譯錯誤：
        public static final int pigeonId = kPigeon2Id; 
        public static final SwerveDriveKinematics kinematics = kKinematics;
        public static final double maxSpeedMetersPerSecond = 4.5; // 實體車最大速度限制 (秒/公尺)，可依實際測試調整
    }
}