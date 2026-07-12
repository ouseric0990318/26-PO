package frc.robot;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
    // 1. 宣告硬體子系統 (這裡以底盤為例，Intake 和 Turret 概念相同)
    private final Drive drive;
    private final Intake intake;
    private final Turret turret;

    // 2. 宣告兩隻 Xbox 手把
    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    // 3. 【核心開關】：決定今天是一個人開還是兩個人開
    // 頂尖心法：也可以寫成 DriverStation.getInstance().isJoystickConnected(1) 自動偵測有沒有插第二隻手把！
    private final boolean isDualDriver = true; 

    public RobotContainer() {
        // 初始化底盤 (實體/模擬自動切換)
        if (Robot.isReal()) {
            drive = new Drive(
            new SwerveModuleIOReal(1, 2, 9, 0.0),
            new SwerveModuleIOReal(3, 4, 10, 0.0),
            new SwerveModuleIOReal(5, 6, 11, 0.0),
            new SwerveModuleIOReal(7, 8, 12, 0.0)
            );
            intake = new Intake(new IntakeIO() {});// 等機構好了寫這兩個 
            turret = new Turret(new TurretIO() {});
        
        } else {
            drive = new Drive(new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim());
            intake = new Intake(new IntakeIO() {}); // 沒寫 Sim 之前先用空介面頂替
            turret = new Turret(new TurretIO() {});
        }

        // 綁定預設指令與按鍵
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* ==========================================================
         * 1. 永遠不變的邏輯：主駕駛 (Driver) 掌控底盤
         * ========================================================== */
        drive.setDefaultCommand(
            new TeleopDrive(
                drive,
                () -> driverController.getLeftY(),  // 前進後退
                () -> driverController.getLeftX(),  // 左右平移
                () -> driverController.getRightX() // 旋轉車身
            )
        );

        // 主駕駛按 Start 鍵可以將陀螺儀歸零 (當開車方向對不準場地時很有用)
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
            // 主駕駛保留 Intake 控制權 (方便一邊開車一邊吸球，反應最快)
            // driverController.a().whileTrue(intake.runIntakeCommand());

            // ❌ 主駕駛的 B 鍵失效，控制權轉移！
            // ⭕ 副駕駛 (Operator) 獨佔砲塔的所有精準控制
            driverController.a().whileTrue(intake.runIntakeCommand());
            
            // 副駕駛按住 B 鍵，砲塔自動追蹤（這裡先塞一個測試用的固定角度，例如轉到 1 弧度）
            operatorController.b().whileTrue(turret.trackTargetCommand(() -> 1.0));
        }
    }
/**
     * 提供給 Robot.java 呼叫，用來取得自動期 (Autonomous) 要執行的 Command
     * @return 自動期指令
     */
public Command getAutonomousCommand() {
        // 我們目前還沒寫 PathPlanner 自動期路徑，所以先暫時回傳 null
        return null;
    }
  }