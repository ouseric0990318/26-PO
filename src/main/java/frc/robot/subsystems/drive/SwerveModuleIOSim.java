package frc.robot.subsystems.drive;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.MathUtil;

public class SwerveModuleIOSim implements SwerveModuleIO {
    // 捨棄所有複雜模擬器，直接用變數儲存當前狀態
    private double driveVelocityRadPerSec = 0.0;
    private double drivePositionRad = 0.0;
    private double steerPositionRad = 0.0;

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // 簡單的時間積分模擬 (dt = 0.02s)
        drivePositionRad += driveVelocityRadPerSec * 0.02;
        
        inputs.drivePositionRad = drivePositionRad;
        inputs.driveVelocityRadPerSec = driveVelocityRadPerSec;
        inputs.steerPositionRad = steerPositionRad;
        inputs.steerAbsolutePositionRad = steerPositionRad;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        // 直接設定速度
        this.driveVelocityRadPerSec = velocityRadPerSec;
    }

    @Override
    public void setSteerAngle(double angleRadians) {
        // 直接賦值角度，模擬伺服馬達反應
        this.steerPositionRad = angleRadians;
    }

    @Override
    public void setDriveVoltage(double volts) {
        // 模擬電壓轉速度 (簡單線性關係)
        this.driveVelocityRadPerSec = volts * 5.0; 
    }

    @Override
    public void setSteerVoltage(double volts) {
        // 模擬電壓轉角度
        this.steerPositionRad += volts * 0.01;
    } 
}