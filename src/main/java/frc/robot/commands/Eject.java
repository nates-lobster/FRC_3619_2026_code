// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.FuelConstants.*;

/**
 * Eject command reverses the mechanism to clear balls.
 */
public class Eject extends Command {
  private final ShooterSubsystem shooter;

  public Eject(ShooterSubsystem shooterSubsystem) {
    shooter = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void execute() {
    // Eject indexer at full power
    shooter.setIndexerPower(-1.0);
    
    // Eject launcher wheels in reverse
    shooter.setLauncherVelocityRPM(-INTAKE_LAUNCHER_RPM);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }
}
