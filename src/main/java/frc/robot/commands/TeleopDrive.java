package frc.robot.commands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DriveConstants; // 如果參數在Constants裡請自行調整引用
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.Drive;

public class TeleopDrive extends Command {
    private final Drive drive;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    public TeleopDrive(Drive drive, 
                       DoubleSupplier translationXSupplier, 
                       DoubleSupplier translationYSupplier, 
                       DoubleSupplier rotationSupplier) {
        this.drive = drive;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        // 1. 讀取數值並套用死區 (Deadband)，防止搖桿沒動時機器人自己發抖
        double xInput = MathUtil.applyDeadband(translationXSupplier.getAsDouble(), OperatorConstants.kDeadband);
        double yInput = MathUtil.applyDeadband(translationYSupplier.getAsDouble(), OperatorConstants.kDeadband);
        double rotInput = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), OperatorConstants.kDeadband);

        // 2. 換算成真實世界的速度 (公尺/秒、弧度/秒)
        // 注意：FRC 的 X 是前進，Y 是橫移。Xbox搖桿往上推是負值，所以前面要加負號
        double maxLinearSpeed = 4.5; // 也可以引用自 Constants
        double maxAngularSpeed = 2.0 * Math.PI;

        double vX = -xInput * maxLinearSpeed;
        double vY = -yInput * maxLinearSpeed;
        double vRot = -rotInput * maxAngularSpeed;

        // 3. 發送給底盤 (預設使用 Field-Relative 場地地圖絕對座標操控，開起來最直覺)
        drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, vRot, drive.getRotation2d()));
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds()); // 結束時安全停車
    }
}