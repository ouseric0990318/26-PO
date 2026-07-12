package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
// 👇 2026 最新版唯一正確的介面名稱：LoggableInputs
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeIO {
    
    // 👇 讓內部類別實作 LoggableInputs
    public static class IntakeIOInputs implements LoggableInputs {
        public double rollerVoltage = 0.0;
        public double rollerVelocityRadPerSec = 0.0;
        public double rollerCurrentAmps = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("RollerVoltage", rollerVoltage);
            table.put("RollerVelocityRadPerSec", rollerVelocityRadPerSec);
            table.put("RollerCurrentAmps", rollerCurrentAmps);
        }

        @Override
        public void fromLog(LogTable table) {
            rollerVoltage = table.get("RollerVoltage", rollerVoltage);
            rollerVelocityRadPerSec = table.get("RollerVelocityRadPerSec", rollerVelocityRadPerSec);
            rollerCurrentAmps = table.get("RollerCurrentAmps", rollerCurrentAmps);
        }
    }

    /** 更新輸入數據 */
    public default void updateInputs(IntakeIOInputs inputs) {}

    /** 控制電壓 */
    public default void setRollerVoltage(double volts) {}
}