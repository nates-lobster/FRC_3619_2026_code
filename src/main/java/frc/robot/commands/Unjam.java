// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;

public class Unjam extends Command {
  private final CANFuelSubsystem fuelSubsystem;

  public Unjam(CANFuelSubsystem fuelSubsystem) {
    this.fuelSubsystem = fuelSubsystem;
    addRequirements(fuelSubsystem);
  }

  @Override
  public void initialize() {
    fuelSubsystem.runUnjam();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    fuelSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
