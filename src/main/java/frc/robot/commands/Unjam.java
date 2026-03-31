// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Unjam command provides a brief reversal to clear any jams in the feeder.
 */
public class Unjam extends Command {
  private final ShooterSubsystem shooter;

  public Unjam(ShooterSubsystem shooterSubsystem) {
    shooter = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void execute() {
    // Reverse slightly to clear jam
    shooter.setIndexerPower(-0.6);
    shooter.setLauncherVelocityRPM(-500);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }
}
