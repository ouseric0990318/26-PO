// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot { 
  
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  public Robot() {
    // 保持建構子清空
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // ─── 【第一步：AdvantageKit 初始化】 ──────────────────────────────────────
    // 注意：這段必須放在 new RobotContainer() 之前，否則子系統的 Log 會全部漏抓！
    
    Logger.recordMetadata("ProjectName", "FRC-2026-Robot"); // 設定專案名稱紀錄

    if (isReal()) {
      // 真機模式：將 Log 數據直接寫入機器人上的 USB 隨身碟，並發送至網絡
      Logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      // 模擬模式：開啟 NT4 伺服器，讓電腦上的 AdvantageScope 可以 Live 連線即時抓取數據
      Logger.addDataReceiver(new NT4Publisher());
    }

    // 正式啟動 Logger
    Logger.start();


    // ─── 【第二步：執行原本的實例化】 ────────────────────────────────────────
    // 當 Logger 活過來後，再開始載入底盤、Intake、Turret 等子系統
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}