package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
  private final Timer timer = new Timer();
  
  // Dashboard override variables
  private boolean usePID;
  private double hardLaunchPower;
  private double hardIndexerReversePower;
  private double hardIndexerForwardPower;
  private double hardReverseSeconds;

  public LaunchSequence(ShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    // Fetch target and modes from SmartDashboard
    targetRPM = SmartDashboard.getNumber("Shooter/Target RPM", LAUNCHING_LAUNCHER_RPM);
    usePID = SmartDashboard.getBoolean("Shooter/Launch PID Enabled", DEFAULT_USE_PID);
    
    // Fetch hardcoded overrides
    hardLaunchPower = SmartDashboard.getNumber("Shooter/Hard Launch Power", HARD_LAUNCH_POWER);
    hardIndexerReversePower = SmartDashboard.getNumber("Shooter/Hard Indexer Reverse Power", HARD_INDEXER_REVERSE_POWER);
    hardIndexerForwardPower = SmartDashboard.getNumber("Shooter/Hard Indexer Forward Power", HARD_INDEXER_FORWARD_POWER);
    hardReverseSeconds = SmartDashboard.getNumber("Shooter/Hard Reverse Seconds", HARD_INDEXER_REVERSE_SECONDS);

    hasReachedSetpoint = false;
    timer.restart();
  }

  @Override
  public void execute() {
    // 1. Run flywheels at target
    if (usePID) {
      shooter.setLauncherVelocityRPM(targetRPM);
    } else {
      shooter.setLauncherPower(hardLaunchPower);
    }

    // 2. Dynamic indexer control based on flywheel speed
    // If we are at the target speed (or using fixed power mode), feed the ball.
    // Otherwise, reverse slightly to allow the flywheel to recover.
    if (shooter.atSetpoint(targetRPM, SHOOTER_RPM_TOLERANCE)) {
      shooter.setIndexerPower(0.60); // 60% forward feeding
    } else {
      shooter.setIndexerPower(-0.20); // 20% backward intake direction
    }
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
