package frc.robot.subsystems.drive;
//定義
public interface SwerveModuleIO {
    
    public static class SwerveModuleIOInputs {
        public double drivePositionRad = 0.0;          
        public double driveVelocityRadPerSec = 0.0;    
        public double steerPositionRad = 0.0;          
        public double steerAbsolutePositionRad = 0.0;  
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {}
    public default void setDriveVoltage(double volts) {}
    public default void setSteerVoltage(double volts) {}
    public default void setSteerAngle(double angleRadians) {}
    public default void setDriveVelocity(double velocityRadPerSec) {}
}