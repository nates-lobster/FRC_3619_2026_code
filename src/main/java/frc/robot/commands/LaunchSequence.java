package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.FuelConstants.*;

/**
 * LaunchSequence coordinates the flywheel spinup and ball indexing.
 * 
 * Logic:
 * 1. Spins up the flywheel using onboard PF velocity control.
 * 2. Reverses the indexer at 10% of target speed while spinning up (clears the path).
 * 3. Advances the indexer at 50% of target speed once the flywheel is at setpoint.
 * 4. Stops everything when the command ends (button release).
 */
public class LaunchSequence extends Command {
  private final ShooterSubsystem shooter;
  private double targetRPM;
  private boolean hasReachedSetpoint;

  public LaunchSequence(ShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    // Fetch target from SmartDashboard if tuned, otherwise use constant
    targetRPM = SmartDashboard.getNumber("Shooter/Target RPM", LAUNCHING_LAUNCHER_RPM);
    hasReachedSetpoint = false;
  }

  @Override
  public void execute() {
    // Always keep the launcher at target velocity
    shooter.setLauncherVelocityRPM(targetRPM);

    // Initial spinup: reverse indexer until flywheel reaches speed
    if (!hasReachedSetpoint) {
      if (shooter.atSetpoint(targetRPM, SHOOTER_RPM_TOLERANCE)) {
        hasReachedSetpoint = true;
      } else {
        // Clear the path while spinning up
        shooter.setIndexerVelocityRPM(targetRPM * INDEXER_REVERSE_RATIO);
        return;
      }
    }

    // Persistent feeding: once setpoint is reached, keep feeding to maintain inertia.
    // If the launcher slows down significantly, we can drop the indexer speed slightly (0.4 ratio),
    // but keep it between 0.4 and 0.5 to keep balls moving.
    double feedRatio = shooter.atSetpoint(targetRPM, SHOOTER_RPM_TOLERANCE) 
        ? INDEXER_FORWARD_RATIO // 0.5
        : 0.4;
    
    shooter.setIndexerVelocityRPM(targetRPM * feedRatio);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the whole system. The launcher will gradually coast to a stop.
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    // This command runs while the button is held.
    return false;
  }
}
