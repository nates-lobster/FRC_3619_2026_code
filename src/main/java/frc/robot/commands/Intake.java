// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.FuelConstants.*;

/**
 * Intake command runs the indexer and flywheel in reverse to pull balls in.
 */
public class Intake extends Command {
  private final ShooterSubsystem shooter;

  public Intake(ShooterSubsystem shooterSubsystem) {
    shooter = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void execute() {
    // Run indexer at intaking power
    shooter.setIndexerPower(INDEXER_INTAKING_PERCENT);
    
    // Spin wheels at a slower RPM for intaking
    shooter.setLauncherVelocityRPM(INTAKE_LAUNCHER_RPM);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }
}
