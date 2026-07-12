package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    
    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0; // 駕駛員 Xbox 搖桿插在 Driver Station 的 Port 0
        public static final int kOperatorControllerPort = 1; // 副駕駛 (手把 2)
        
        // 搖桿死區 (Deadband)
        public static final double kDeadband = 0.1;
        
    }

    /** 底盤相關物理與控制參數 */
    public static final class DriveConstants {
        public static final double trackWidth = Units.inchesToMeters(24.0); 
        public static final double wheelBase = Units.inchesToMeters(24.0);  
        public static final int pigeonId = 0;

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),  
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), 
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), 
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0) 
        );

        public static final double wheelRadius = Units.inchesToMeters(2.0); 
        public static final double driveGearRatio = 6.75; 
        public static final double steerGearRatio = 150.0 / 7.0; 

        public static final double maxSpeedMetersPerSecond = 4.5; 
        public static final double maxAngularVelocityRadPerSec = 2.0 * Math.PI; 
    }
}