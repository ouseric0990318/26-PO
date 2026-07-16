package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public double yawPositionRad = 0.0;      // 車頭朝向 (弧度)
        public double yawVelocityRadPerSec = 0.0;// 旋轉角速度 (弧度/秒)
    }

    /** 更新感測器數據 */
    public default void updateInputs(GyroIOInputs inputs) {}
}