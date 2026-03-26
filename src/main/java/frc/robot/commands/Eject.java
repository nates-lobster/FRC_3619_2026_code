// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

public class Eject extends Command {
  private final CANFuelSubsystem fuelSubsystem;
  private boolean isUnjamming = false;
  private final edu.wpi.first.wpilibj.Timer unjamTimer = new edu.wpi.first.wpilibj.Timer();

  public Eject(CANFuelSubsystem fuelSystem) {
    addRequirements(fuelSystem);
    this.fuelSubsystem = fuelSystem;
  }

  // Called when the command is initially scheduled. Set the rollers to the
  // appropriate values for ejecting
  @Override
  public void initialize() {
    isUnjamming = false;
    fuelSubsystem
        .setIntakeLauncherRoller(
             -1 * SmartDashboard.getNumber("Intaking intake roller value", INTAKE_EJECT_PERCENT));
    fuelSubsystem.setFeederRoller(SmartDashboard.getNumber("Intaking intake roller value", INDEXER_LAUNCHING_PERCENT));
  }

  // Called every time the scheduler runs while the command is scheduled. This
  // command doesn't require updating any values while running
  @Override
  public void execute() {
    double feederTarget = SmartDashboard.getNumber("Intaking intake roller value", INDEXER_LAUNCHING_PERCENT);
    double launcherTarget = -1 * SmartDashboard.getNumber("Intaking intake roller value", INTAKE_EJECT_PERCENT);

    if (isUnjamming) {
      if (unjamTimer.hasElapsed(1.0)) {
        isUnjamming = false;
        fuelSubsystem.setFeederRoller(feederTarget);
        fuelSubsystem.setIntakeLauncherRoller(launcherTarget);
      }
    } else if (fuelSubsystem.isIndexerJammed(feederTarget)) {
      isUnjamming = true;
      unjamTimer.restart();
      fuelSubsystem.runUnjam();
    }
  }

  // Called once the command ends or is interrupted. Stop the rollers
  @Override
  public void end(boolean interrupted) {
    fuelSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
