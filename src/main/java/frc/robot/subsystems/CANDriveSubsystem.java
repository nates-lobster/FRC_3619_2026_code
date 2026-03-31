// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OperatorConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final DifferentialDrive drive;
  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDriveOdometry odometry;

  // Gyro for heading - if you don't have one, rotation will use encoder-based estimation
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private final Field2d field = new Field2d();

  private final Timer turboTimer = new Timer();
  private static final double TURBO_DURATION = 1.5;

  public CANDriveSubsystem() {
    // Create NEO brushless motors
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    // Set CAN timeout
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Build config for followers first
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    followerConfig.voltageCompensation(12);
    followerConfig.idleMode(IdleMode.kBrake);

    followerConfig.follow(leftLeader);
    leftFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followerConfig.follow(rightLeader);
    rightFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Leader config with encoder conversion factors
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    leaderConfig.voltageCompensation(12);
    leaderConfig.idleMode(IdleMode.kBrake);
    leaderConfig.encoder
        .positionConversionFactor(POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

    // Right side: inverted so positive values drive forward on both sides
    leaderConfig.inverted(true);
    rightLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Left side: not inverted
    leaderConfig.inverted(false);
    leftLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Get built-in NEO encoders
    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();
    resetEncoders();

    // Set up differential drive
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set up kinematics and odometry
    kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
    odometry = new DifferentialDriveOdometry(
        getHeading(),
        leftEncoder.getPosition(),
        rightEncoder.getPosition()
    );

    SmartDashboard.putData("Field", field);
    SmartDashboard.putNumber("TurnAround reverse speed", TURNAROUND_REVERSE_SPEED);
    SmartDashboard.putNumber("TurnAround reverse seconds", TURNAROUND_REVERSE_SECONDS);
    SmartDashboard.putNumber("TurnAround spin speed", TURNAROUND_SPIN_SPEED);
    SmartDashboard.putNumber("TurnAround spin seconds", TURNAROUND_SPIN_SECONDS);

    // Configure PathPlanner AutoBuilder
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getChassisSpeeds,
          (speeds, feedforwards) -> driveFromChassisSpeeds(speeds),
          new PPLTVController(0.02),
          config,
          () -> false, // Set to true if on red alliance and paths need to flip
          this
      );
    } catch (Exception e) {
      System.err.println("[PathPlanner] Failed to load robot config from GUI: " + e.getMessage());
    }
  }

  @Override
  public void periodic() {
    // Update odometry each loop
    odometry.update(getHeading(), leftEncoder.getPosition(), rightEncoder.getPosition());
    field.setRobotPose(getPose());

    // Turbo timer management
    if (turboTimer.hasElapsed(TURBO_DURATION)) {
      turboTimer.stop();
      turboTimer.reset();
    }
    SmartDashboard.putBoolean("Turbo Active", isTurboActive());
    SmartDashboard.putNumber("Left Encoder (m)", leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder (m)", rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Velocity (m/s)", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Velocity (m/s)", rightEncoder.getVelocity());
  }

  // ---- Odometry / Pose ----

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(getHeading(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public Rotation2d getHeading() {
    // Calculate heading based on wheel encoders since gyro is not present/functional.
    // Right wheel moving forward and left moving backward causes leftward (positive) rotation.
    double headingRadians = (rightEncoder.getPosition() - leftEncoder.getPosition()) / TRACK_WIDTH_METERS;
    return Rotation2d.fromRadians(headingRadians);
  }

  // ---- Chassis Speeds (required by PathPlanner) ----

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity())
    );
  }

  public void driveFromChassisSpeeds(ChassisSpeeds speeds) {
    var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    // Normalize in case one side exceeds max
    wheelSpeeds.desaturate(4.5); // ~4.5 m/s max for NEOs on KOP
    // Convert m/s back to -1..1 range for arcadeDrive equivalent
    double left = wheelSpeeds.leftMetersPerSecond / 4.5;
    double right = wheelSpeeds.rightMetersPerSecond / 4.5;
    leftLeader.set(left);
    rightLeader.set(right);
  }

  // ---- Teleop Drive ----

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void driveArcade(double xSpeed, double zRotation, boolean squareInputs) {
    drive.arcadeDrive(xSpeed, zRotation, squareInputs);
  }

  // ---- Turbo ----

  public void activateTurbo() {
    turboTimer.restart();
  }

  public boolean isTurboActive() {
    return turboTimer.isRunning();
  }

  public double getDriveScaling() {
    return isTurboActive() ? 1.0 : DRIVE_SCALING;
  }
}
