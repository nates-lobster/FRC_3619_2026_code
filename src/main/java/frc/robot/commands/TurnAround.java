// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;

import static frc.robot.Constants.DriveConstants.*;

public class TurnAround extends SequentialCommandGroup {
  public TurnAround(CANDriveSubsystem driveSubsystem) {
    double reverseSpeedPercent = SmartDashboard.getNumber("TurnAround reverse speed", TURNAROUND_REVERSE_SPEED);
    double reverseSeconds = SmartDashboard.getNumber("TurnAround reverse seconds", TURNAROUND_REVERSE_SECONDS);
    double spinSpeedPercent = SmartDashboard.getNumber("TurnAround spin speed", TURNAROUND_SPIN_SPEED);
    double spinSeconds = SmartDashboard.getNumber("TurnAround spin seconds", TURNAROUND_SPIN_SECONDS);

    double reverseSpeed = reverseSpeedPercent / 100.0;
    double spinSpeed = spinSpeedPercent / 100.0;

    addCommands(
        new AutoDrive(driveSubsystem, -reverseSpeed, 0.0).withTimeout(reverseSeconds),
        new AutoDrive(driveSubsystem, 0.0, spinSpeed).withTimeout(spinSeconds));
  }
}
