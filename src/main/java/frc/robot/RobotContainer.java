package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveModuleIOReal;
import frc.robot.subsystems.drive.SwerveModuleIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    // 1. 宣告硬體子系統
    private final Drive drive;
    private final Intake intake;
    private final Turret turret;

    // 2. 宣告兩隻 Xbox 手把
    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    // 3. 【核心開關】：決定今天是一個人開還是兩個人開
    private final boolean isDualDriver = true; 

    public RobotContainer() {
        // ====== 🟢 步驟 1：先完整初始化所有底盤與子系統 ======
        if (Robot.isReal()) {
            drive = new Drive(
                new SwerveModuleIOReal(
                    Constants.DriveConstants.kFLDriveId, 
                    Constants.DriveConstants.kFLSteerId, 
                    Constants.DriveConstants.kFLEncoderId, 
                    Constants.DriveConstants.kFLOffset
                ),
                new SwerveModuleIOReal(
                    Constants.DriveConstants.kFRDriveId, 
                    Constants.DriveConstants.kFRSteerId, 
                    Constants.DriveConstants.kFREncoderId, 
                    Constants.DriveConstants.kFROffset
                ),
                new SwerveModuleIOReal(
                    Constants.DriveConstants.kBLDriveId, 
                    Constants.DriveConstants.kBLSteerId, 
                    Constants.DriveConstants.kBLEncoderId, 
                    Constants.DriveConstants.kBLOffset
                ),
                new SwerveModuleIOReal(
                    Constants.DriveConstants.kBRDriveId, 
                    Constants.DriveConstants.kBRSteerId, 
                    Constants.DriveConstants.kBREncoderId, 
                    Constants.DriveConstants.kBROffset
                )
            );
            intake = new Intake(new IntakeIO() {}); // 等機構好了寫這兩個 
            turret = new Turret(new TurretIO() {});
        
        } else {
            drive = new Drive(new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim());
            intake = new Intake(new IntakeIO() {}); // 沒寫 Sim 之前先用空介面頂替
            turret = new Turret(new TurretIO() {});
        }

        // ====== 🟢 步驟 2：底盤 new 完（大腦配置成功）後，再建立路徑選擇器 ======
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        // 綁定預設指令與按鍵
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* ==========================================================
         * 1. 主駕駛 (Driver) 掌控底盤
         * ========================================================== */
        drive.setDefaultCommand(
            new TeleopDrive(
                drive,
                () -> driverController.getLeftY(),  // 前進後退
                () -> driverController.getLeftX(),  // 左右平移
                () -> driverController.getRightX()  // 旋轉車身
            )
        );

        // 主駕駛按 Start 鍵將陀螺儀歸零
        driverController.start().onTrue(new InstantCommand(() -> drive.zeroGyro(), drive));

        /* ==========================================================
         * 2. 動態分流邏輯：Intake 與 Turret 的控制權分配
         * ========================================================== */
        if (!isDualDriver) {
            /* ─── 【單人模式控制】 ─── */
            driverController.a().whileTrue(intake.runIntakeCommand());
            driverController.b().whileTrue(turret.centerTurretCommand());
            
        } else {
            /* ─── 【雙人模式控制】 ─── */
            driverController.a().whileTrue(intake.runIntakeCommand());
            
            // 副駕駛按住 B 鍵，砲塔自動追蹤
            operatorController.b().whileTrue(turret.trackTargetCommand(() -> 1.0));
        }
    }

    /**
     * 提供給 Robot.java 呼叫，用來取得自動期 (Autonomous) 要執行的 Command
     * @return 自動期指令
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}