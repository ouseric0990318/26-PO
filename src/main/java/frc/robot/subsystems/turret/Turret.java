package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIO.TurretIOInputs inputs = new TurretIO.TurretIOInputs();

    private static final double MIN_ANGLE_RAD = Math.toRadians(-170.0);
    private static final double MAX_ANGLE_RAD = Math.toRadians(170.0);

    public Turret(TurretIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
    }

    public void setTargetAngle(double targetRad) {
        double clampedTarget = MathUtil.clamp(targetRad, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
        io.setTargetPositionRad(clampedTarget);
        Logger.recordOutput("Turret/TargetAngleRad", clampedTarget);
    }

    public void stop() {
        io.setVoltage(0.0);
    }

    public Command centerTurretCommand() {
        return this.run(() -> setTargetAngle(0.0)).withName("CenterTurret");
    }

    public Command trackTargetCommand(java.util.function.DoubleSupplier targetAngleSupplier) {
        return this.run(() -> setTargetAngle(targetAngleSupplier.getAsDouble()))
                   .finallyDo((interrupted) -> stop())
                   .withName("TrackTarget");
    }
}