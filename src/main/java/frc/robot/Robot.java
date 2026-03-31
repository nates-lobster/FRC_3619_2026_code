// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final Timer m_shiftTimer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Used to track usage of Kitbot code, please do not remove.
    HAL.report(tResourceType.kResourceType_Framework, 10);
    HttpCamera backCamera = new HttpCamera("Back Camera", "http://10.36.19.11:1181/?action=stream");
    HttpCamera frontRightCamera = new HttpCamera("Front Right Camera", "http://10.36.19.11:1182/?action=stream");
    HttpCamera frontLeftCamera = new HttpCamera("Front Left Camera", "http://10.36.19.11:1183/?action=stream");
    CameraServer.addCamera(backCamera);
    CameraServer.addCamera(frontRightCamera);
    CameraServer.addCamera(frontLeftCamera);
    
    // Shift Timer initialization
    SmartDashboard.putNumber("Manual Shift Duration (s)", 150.0);
    SmartDashboard.putBoolean("Reset/Start Manual Shift", false);
    SmartDashboard.putString("Current Shift", "Ready");
    m_shiftTimer.stop();
    m_shiftTimer.reset();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());

    ShooterSubsystem shooterSubsystem = m_robotContainer.getShooterSubsystem();
    SmartDashboard.putNumber("Live Launcher Velocity", shooterSubsystem.getVelocityRPM());
    SmartDashboard.putNumber("Live Indexer Velocity", shooterSubsystem.getLiveIndexerVelocity());

    // Shift timer logic
    double matchTime = Timer.getMatchTime();
    String shiftName = "NO MATCH";
    double timeInShift = 0;
    boolean isMatchActive = matchTime > 0;

    if (isAutonomous()) {
      shiftName = "AUTO";
      timeInShift = matchTime;
    } else if (isTeleop() && isMatchActive) {
      // Logic based on the provided "MATCH Timeframe" table
      // Starts at 2:20 (140s)
      if (matchTime > 130) {
        shiftName = "TRANSITION";
        timeInShift = matchTime - 130;
      } else if (matchTime > 105) {
        shiftName = "SHIFT 1";
        timeInShift = matchTime - 105;
      } else if (matchTime > 80) {
        shiftName = "SHIFT 2";
        timeInShift = matchTime - 80;
      } else if (matchTime > 55) {
        shiftName = "SHIFT 3";
        timeInShift = matchTime - 55;
      } else if (matchTime > 30) {
        shiftName = "SHIFT 4";
        timeInShift = matchTime - 30;
      } else {
        shiftName = "END GAME";
        timeInShift = matchTime;
      }
    } else {
      // Manual Practice Timer Logic
      if (SmartDashboard.getBoolean("Reset/Start Manual Shift", false)) {
        m_shiftTimer.restart();
        SmartDashboard.putBoolean("Reset/Start Manual Shift", false);
      }
      
      double manualDuration = SmartDashboard.getNumber("Manual Shift Duration (s)", 150.0);
      if (m_shiftTimer.isRunning()) {
        double remaining = manualDuration - m_shiftTimer.get();
        shiftName = (remaining <= 0) ? "SHIFT OVER" : "PRACTICE SHIFT";
        timeInShift = Math.max(0, remaining);
      } else {
        shiftName = "READY (MANUAL)";
        timeInShift = manualDuration;
      }
    }

    SmartDashboard.putString("Current Shift", shiftName);
    SmartDashboard.putNumber("Shift Time Remaining (s)", timeInShift);
    SmartDashboard.putBoolean("Shift Ending Soon", isMatchActive && (timeInShift < 5.0 && !shiftName.equals("END GAME")));
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
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
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
