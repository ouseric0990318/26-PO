package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.DriveConstants;

public class SwerveModuleIOReal implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder absoluteEncoder;

    public SwerveModuleIOReal(int driveId, int steerId, int encoderId, double steerOffset) {
        
        // 🟢 1. 宣告硬體，並全面綁定高速 CANivore 網卡
        driveMotor = new TalonFX(driveId, DriveConstants.kCANbusName);
        steerMotor = new TalonFX(steerId, DriveConstants.kCANbusName);
        absoluteEncoder = new CANcoder(encoderId, DriveConstants.kCANbusName);

        // 🟢 2. 配置驅動馬達 (Kraken X60)
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        
        // 電流與中性模式設定
        driveConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.kDriveCurrentLimit;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.kDriveStatorCurrentLimit;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // 煞車模式

        // 驅動 PID 參數
        driveConfig.Slot0.kP = DriveConstants.kDriveKP;
        driveConfig.Slot0.kI = DriveConstants.kDriveKI;
        driveConfig.Slot0.kD = DriveConstants.kDriveKD;
        driveConfig.Slot0.kS = DriveConstants.kDriveKS;
        driveConfig.Slot0.kV = DriveConstants.kDriveKV;

        driveMotor.getConfigurator().apply(driveConfig);


        // 🟢 3. 配置轉向馬達 (Talon FX) 與 FusedCANCoder 融合
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        
        // 核心設定：啟動 FusedCANCoder 並綁定對應的編碼器 ID
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        steerConfig.Feedback.FeedbackRemoteSensorID = encoderId;
        
        // 設定轉向馬達與編碼器之間的物理減速比
        steerConfig.Feedback.RotorToSensorRatio = DriveConstants.kSteerGearRatio;
        
        // 設定絕對角度偏移 (Offset)
        steerConfig.Feedback.SensorToMechanismRatio = 1.0; 
        
        // 電流與中性模式設定
        steerConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.kSteerCurrentLimit;
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.kSteerStatorCurrentLimit;
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // 轉向 PID 參數 (Slot 0)
        steerConfig.Slot0.kP = DriveConstants.kSteerKP;
        steerConfig.Slot0.kI = DriveConstants.kSteerKI;
        steerConfig.Slot0.kD = DriveConstants.kSteerKD;

        steerMotor.getConfigurator().apply(steerConfig);
    }
}