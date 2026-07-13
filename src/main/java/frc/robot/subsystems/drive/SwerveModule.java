package frc.robot.subsystems.drive;
//負責計算
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleIO.SwerveModuleIOInputs inputs = new SwerveModuleIO.SwerveModuleIOInputs();

    public SwerveModule(SwerveModuleIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // 取得當前模組的角度
        Rotation2d currentAngle = new Rotation2d(inputs.steerPositionRad);
        desiredState.optimize(currentAngle);
        io.setSteerAngle(desiredState.angle.getRadians());

        // 計算並下發目標速度
        double targetModuleRadPerSec = desiredState.speedMetersPerSecond / DriveConstants.wheelRadius;
        double targetMotorRadPerSec = targetModuleRadPerSec * DriveConstants.driveGearRatio;
        io.setDriveVelocity(targetMotorRadPerSec);
    }

    public SwerveModulePosition getPosition() {
        double distanceMeters = (inputs.drivePositionRad / DriveConstants.driveGearRatio) * DriveConstants.wheelRadius;
        return new SwerveModulePosition(distanceMeters, new Rotation2d(inputs.steerPositionRad));
    }
    //  這是新加入的方法：讓底盤能抓到這一顆模組的「即時速度」與「當前角度」
    public SwerveModuleState getState() {
        // 1. 將馬達的角速度 (Rad/Sec) 透過減速比與輪子半徑，轉換為真正的地表平移速度 (m/s)
        //  提示：請確認你的 inputs 裡面的變數名稱是不是叫 driveVelocityRadPerSec
        double speedMetersPerSecond = (inputs.driveVelocityRadPerSec / DriveConstants.driveGearRatio) * DriveConstants.wheelRadius;
        
        // 2. 取得當前的轉向角度
        Rotation2d currentAngle = new Rotation2d(inputs.steerPositionRad);
        
        // 3. 打包成 SwerveModuleState 回傳
        return new SwerveModuleState(speedMetersPerSecond, currentAngle);
    }
}