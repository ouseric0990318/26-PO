package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    // 宣告四個 Swerve 模組
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    
    // 升級為 Phoenix 6 Pigeon2 陀螺儀
    private final Pigeon2 gyro; 

    // 新增：底盤里程計，用來動態追蹤機器人在場上的絕對座標 (X, Y, Theta)
    private final SwerveDriveOdometry odometry;
    // 新增：用來在模擬模式下累加車頭角度的變數（單位：弧度）
    private double m_simYawRad = 0;

    /** 底盤建構子，接收四個 IO 實作（支援 Real 或 Sim） */
    public Drive(SwerveModuleIO flIO, SwerveModuleIO frIO, SwerveModuleIO blIO, SwerveModuleIO brIO) {
        this.frontLeft = new SwerveModule(flIO);
        this.frontRight = new SwerveModule(frIO);
        this.backLeft = new SwerveModule(blIO);
        this.backRight = new SwerveModule(brIO);

        // 初始化 Pigeon2 (硬體 ID 定義在 DriveConstants 中)
        this.gyro = new Pigeon2(DriveConstants.pigeonId);
        zeroGyro(); // 開機時自動將前方校正為 0 度

        // 初始化里程計
        this.odometry = new SwerveDriveOdometry(
            DriveConstants.kinematics,
            getRotation2d(),
            getModulePositions()
        );
        // ======= PathPlanner AutoBuilder 配置 =======
        try {
            // 自動從 src/main/deploy/pathplanner/settings.json 讀取你在 GUI 裡填寫的重量、MOI 等參數
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                this::getPose,                // 1. 機器人目前座標供應器 (Supplier)
                this::resetOdometry,          // 2. 重設座標的消費者 (Consumer)
                this::getRobotRelativeSpeeds, // 3. 晶片相對速度供應器
                this::drive,                  // 4. 驅動底盤控制方法 (傳入 Robot-Relative 速度)
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0), // 平移 P-ID（真機依狀況調整，模擬用 5.0 很穩）
                    new PIDConstants(5.0, 0.0, 0.0)  // 旋轉 P-ID
                ),
                config,                       // 5. 讀取出的機器人實體參數
                () -> {
                    // 6. 紅方自動路徑鏡像翻轉邏輯
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this                          // 7. 綁定此底盤子系統需求
            );
        } catch (Exception e) {
            DriverStation.reportError("PathPlanner AutoBuilder 配置失敗: " + e.getMessage(), e.getStackTrace());
        }
    }

    /** * 核心驅動方法：接收目標 ChassisSpeeds 並分配給四個輪子
     * @param speeds 目標底盤速度
     */
    public void drive(ChassisSpeeds speeds) {
        // 將底盤速度轉換為四個模組的狀態 (速度與角度)
        SwerveModuleState[] states = DriveConstants.kinematics.toSwerveModuleStates(speeds);
        
        // 確保不超過最大設定速度（解飽和演算法）
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxSpeedMetersPerSecond);

        // 將狀態發送給各個模組
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    /** 獲取四個模組當前的位置與角度陣列（提供給里程計更新使用） */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    /** 取得目前機器人的角度（提供給 Field-Relative 控制與里程計使用） */
    public Rotation2d getRotation2d() {
        if (Robot.isReal()) {
        return gyro.getRotation2d(); // 真機模式：讀取真實 Pigeon2
        } else {
        return new Rotation2d(m_simYawRad); // 模擬模式：回傳我們自己累加的角度
        }
    }

    /** 取得當前機器人在賽場上的絕對動態姿態座標 (Pose2d) */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /** 手動重設里程計座標（通常在自動階段開局、或配合視覺校正時使用） */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    /** 將陀螺儀角度歸零（校正前方） */
    public void zeroGyro() {
        // Phoenix 6 API 使用 setYaw(0) 來歸零角度
        gyro.setYaw(0);
    }

    @Override
    public void periodic() {
        // 1. 重要：每個週期強制更新個別模組內部的 IO 資料（呼叫 updateInputs）
        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();

        // 2. 融合陀螺儀與輪子編碼器數據，動態更新場地絕對座標
        odometry.update(getRotation2d(), getModulePositions());

        // 3. AdvantageKit Telemetry 自動紀錄
        // 這樣你在 AdvantageScope 裡拉開 Odometry/RobotPose，就能即時看見 2D/3D 機器人在場上的精準走位
        Logger.recordOutput("Odometry/RobotPose", odometry.getPoseMeters());
        
        // 額外記錄陀螺儀原始角度，除錯時非常好用
        Logger.recordOutput("Drive/GyroAngleRad", getRotation2d().getRadians());
    }
    @Override
    public void simulationPeriodic() {
    // 透過 Kinematics 把四個輪子的即時狀態逆推回底盤目前的實際速度
    ChassisSpeeds chassisSpeeds = DriveConstants.kinematics.toChassisSpeeds(
        frontLeft.getState(), // 如果你的 SwerveModule 取得狀態的方法叫 getModuleState() 記得同步修改
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState()    
    );

    // 每一個週期 (20ms = 0.02秒) 根據底盤的旋轉角速度 (omega)，動態累積車頭轉動的弧度
    m_simYawRad += chassisSpeeds.omegaRadiansPerSecond * 0.02;
    }
    /**  取得機器人相對座標系的速度（PathPlanner 核心必備） */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kinematics.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        );
    }
}