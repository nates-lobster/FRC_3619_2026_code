// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity. 
 */
public final class Constants {

  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 3;
    public static final int RIGHT_LEADER_ID = 2;
    public static final int RIGHT_FOLLOWER_ID = 4;

    // NEO motors are brushless
    // Current limit for drivetrain motors (NEOs can handle up to 80A, 40A is conservative for drivetrain)
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;

    // ---- ROBOT GEOMETRY - TUNE THESE FOR YOUR ROBOT ----
    // Wheel diameter in meters. Common FRC sizes: 6" = 0.1524m, 4" = 0.1016m
    public static final double WHEEL_DIAMETER_METERS = 0.1524; // 6 inch wheels
    // Track width: distance between left and right wheels in meters (measure center-to-center)
    public static final double TRACK_WIDTH_METERS = 0.56; // tune this measure your robot
    // Gear ratio between motor shaft and wheel (e.g. 8.45:1 is common for NEO on KOP chassis)
    public static final double DRIVE_GEAR_RATIO = 8.45;

    // Encoder conversion factor: converts motor rotations → meters at the wheel
    public static final double POSITION_CONVERSION_FACTOR =
        (WHEEL_DIAMETER_METERS * Math.PI) / DRIVE_GEAR_RATIO;
    // Velocity conversion: motor RPM → meters per second
    public static final double VELOCITY_CONVERSION_FACTOR =
        POSITION_CONVERSION_FACTOR / 60.0;

    public static final double TURNAROUND_REVERSE_SPEED = 35.0;
    public static final double TURNAROUND_REVERSE_SECONDS = 0.60;
    public static final double TURNAROUND_SPIN_SPEED = 55.0;
    public static final double TURNAROUND_SPIN_SECONDS = 0.90;
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int LEFT_INTAKE_LAUNCHER_MOTOR_ID = 5;
    public static final int RIGHT_INTAKE_LAUNCHER_MOTOR_ID = 6;
    public static final int INDEXER_MOTOR_ID = 8;

    // Current limit for fuel mechanism motors.
    public static final int INDEXER_MOTOR_CURRENT_LIMIT = 80;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 80;

    // All values likely need to be tuned based on your robot
    public static final double INDEXER_INTAKING_PERCENT = -0.6; // Flipped sign to fix intake direction
    public static final double INDEXER_LAUNCHING_PERCENT = 1.0;
    public static final double INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT = 0.5;

    public static final double INTAKE_INTAKING_PERCENT = 0.8;
    public static final double LAUNCHING_LAUNCHER_RPM = 3000.0;
    public static final double INTAKE_LAUNCHER_RPM = 1000.0;
    public static final double INTAKE_EJECT_PERCENT = -0.8;

    public static final double SPIN_UP_SECONDS = 0.3;
    
    // Hardcoded Launch Mode Constants
    public static final boolean DEFAULT_USE_PID = false;
    public static final double HARD_LAUNCH_POWER = 0.7;
    public static final double HARD_INDEXER_REVERSE_POWER = -0.4;
    public static final double HARD_INDEXER_FORWARD_POWER = 0.6;
    public static final double HARD_INDEXER_REVERSE_SECONDS = 0.3;

    // Shooter PID/PF Constants
    // We use PF (Proportional + Feedforward) for flywheels because:
    // 1. FF provides the baseline voltage needed for the target RPM.
    // 2. P corrects for small errors/dips when a ball is launched.
    // 3. I-term is avoided to prevent windup/overshoot on a high-inertia wheel.
    public static final double SHOOTER_P = 0.0001; 
    public static final double SHOOTER_FF = 0.00017;
    public static final double INDEXER_FF = 0.0002;
    
    public static final double SHOOTER_RPM_TOLERANCE = 100.0;

    // Indexer Ratios (as percentage of launcher target RPM)
    public static final double INDEXER_REVERSE_RATIO = -0.1;
    public static final double INDEXER_FORWARD_RATIO = 0.5;
  }

  public static final class ClimbConstants {
    // Motor controller IDs for Climb motor
    public static final int CLIMBER_MOTOR_ID = 7;

    // Current limit for climb motor
    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 30;
    // Percentage to power the motor both up and down
    public static final double CLIMBER_MOTOR_DOWN_PERCENT = -0.8;
    public static final double CLIMBER_MOTOR_UP_PERCENT = 0.8;
  }

  public static final class OperatorConstants {

    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and beign difficult to control
    public static final double DRIVE_SCALING = 0.7;
    public static final double ROTATION_SCALING = 0.8;
  } 
}
