package frc.robot.subsystems.drive;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveModuleIOReal implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;

    private final PositionVoltage steerRequest = new PositionVoltage(0);
    private final VelocityVoltage driveRequest = new VelocityVoltage(0);

    public SwerveModuleIOReal(int driveId, int steerId, int encoderId, double steerOffset) {
        driveMotor = new TalonFX(driveId);
        steerMotor = new TalonFX(steerId);
        steerEncoder = new CANcoder(encoderId);
        // (硬體 PID 與  Offset 設定可在此處擴充)
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionRad = driveMotor.getPosition().getValueAsDouble() * 2.0 * Math.PI;
        inputs.driveVelocityRadPerSec = driveMotor.getVelocity().getValueAsDouble() * 2.0 * Math.PI;
        inputs.steerPositionRad = steerMotor.getPosition().getValueAsDouble() * 2.0 * Math.PI;
        inputs.steerAbsolutePositionRad = steerEncoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI;
    }

    @Override
    public void setSteerAngle(double angleRadians) {
        steerMotor.setControl(steerRequest.withPosition(angleRadians / (2.0 * Math.PI)));
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveMotor.setControl(driveRequest.withVelocity(velocityRadPerSec / (2.0 * Math.PI)));
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setSteerVoltage(double volts) {
        steerMotor.setVoltage(volts);
    }
}