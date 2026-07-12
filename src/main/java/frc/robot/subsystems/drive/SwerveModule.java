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
}