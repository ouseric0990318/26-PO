package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface TurretIO {
    
    public static class TurretIOInputs implements LoggableInputs {
        public double turretPositionRad = 0.0;
        public double turretVelocityRadPerSec = 0.0;
        public double turretAppliedVolts = 0.0;
        public double turretCurrentAmps = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("TurretPositionRad", turretPositionRad);
            table.put("TurretVelocityRadPerSec", turretVelocityRadPerSec);
            table.put("TurretAppliedVolts", turretAppliedVolts);
            table.put("TurretCurrentAmps", turretCurrentAmps);
        }

        @Override
        public void fromLog(LogTable table) {
            turretPositionRad = table.get("TurretPositionRad", turretPositionRad);
            turretVelocityRadPerSec = table.get("TurretVelocityRadPerSec", turretVelocityRadPerSec);
            turretAppliedVolts = table.get("TurretAppliedVolts", turretAppliedVolts);
            turretCurrentAmps = table.get("TurretCurrentAmps", turretCurrentAmps);
        }
    }

    public default void updateInputs(TurretIOInputs inputs) {}
    public default void setTargetPositionRad(double positionRad) {}
    public default void setVoltage(double volts) {}
}