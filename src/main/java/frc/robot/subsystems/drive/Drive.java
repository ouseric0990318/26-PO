package frc.robot.subsystems.drive;

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
        // Phoenix 6 的 Pigeon2 內建支援直接回傳 WPILib 的 Rotation2d 物件
        return gyro.getRotation2d();
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
}