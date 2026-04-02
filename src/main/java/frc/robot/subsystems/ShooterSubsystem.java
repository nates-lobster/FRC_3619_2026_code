package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FuelConstants.*;

/**
 * ShooterSubsystem handles the flywheel launcher and indexer mechanism.
 * 
 * Uses Spark MAX onboard closed-loop velocity control (kVelocity mode).
 * This is preferred over running PID on the roboRIO because:
 * 1. It operates at 1kHz directly on the motor controller, providing much faster response.
 * 2. It reduces CAN bus traffic as setpoints are sent less frequently than sensor data.
 * 
 * We use a PF (Proportional + Feedforward) controller for the flywheel:
 * - Feedforward (FF) provides the baseline voltage required to sustain a target RPM.
 * - Proportional (P) corrects for small dips in speed (e.g., when a ball is launched).
 * - Integral/Derivative are avoided to keep tuning simple and prevent overshoot/oscillation
 *   in high-inertia flywheel systems.
 */
public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax leftFlywheel;
  private final SparkMax rightFlywheel;
  private final SparkMax indexer;

  private final SparkClosedLoopController leftController;
  private final SparkClosedLoopController rightController;
  private final SparkClosedLoopController indexerController;

  private double lastP, lastFF;

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    leftFlywheel = new SparkMax(LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    rightFlywheel = new SparkMax(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    indexer = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);

    // Set CAN timeout to ensure configs are applied reliably
    leftFlywheel.setCANTimeout(250);
    rightFlywheel.setCANTimeout(250);
    indexer.setCANTimeout(250);

    // Flywheel Configuration
    SparkMaxConfig flywheelConfig = new SparkMaxConfig();
    flywheelConfig
        .smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT)
        .voltageCompensation(12)
        .idleMode(IdleMode.kCoast);
    
    // Set PF gains for the flywheel
    flywheelConfig.closedLoop
        .p(SHOOTER_P);
    flywheelConfig.closedLoop.feedForward.kV(SHOOTER_FF);

    // Apply config to flywheels (right is inverted in this setup)
    rightFlywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    flywheelConfig.inverted(true);
    leftFlywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Indexer Configuration
    SparkMaxConfig indexerConfig = new SparkMaxConfig();
    indexerConfig
        .smartCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake);
    
    // Indexer also uses velocity control during launch
    indexerConfig.closedLoop
        .p(SHOOTER_P);
    indexerConfig.closedLoop.feedForward.kV(INDEXER_FF);

    indexer.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Capture controllers for closed-loop control
    leftController = leftFlywheel.getClosedLoopController();
    rightController = rightFlywheel.getClosedLoopController();
    indexerController = indexer.getClosedLoopController();

    // Preferences/SmartDashboard setup for live tuning
    // Use Preferences if available, otherwise fallback to Constants
    Preferences.initDouble("Shooter/P", SHOOTER_P);
    Preferences.initDouble("Shooter/FF", SHOOTER_FF);
    Preferences.initDouble("Shooter/Target RPM", LAUNCHING_LAUNCHER_RPM);

    SmartDashboard.putNumber("Shooter/P", Preferences.getDouble("Shooter/P", SHOOTER_P));
    SmartDashboard.putNumber("Shooter/FF", Preferences.getDouble("Shooter/FF", SHOOTER_FF));
    SmartDashboard.putNumber("Shooter/Target RPM", Preferences.getDouble("Shooter/Target RPM", LAUNCHING_LAUNCHER_RPM));

    lastP = Preferences.getDouble("Shooter/P", SHOOTER_P);
    lastFF = Preferences.getDouble("Shooter/FF", SHOOTER_FF);

    // Hardcoded control switches and variables
    SmartDashboard.putBoolean("Shooter/Launch PID Enabled", DEFAULT_USE_PID);
    SmartDashboard.putNumber("Shooter/Hard Launch Power", HARD_LAUNCH_POWER);
    SmartDashboard.putNumber("Shooter/Hard Indexer Reverse Power", HARD_INDEXER_REVERSE_POWER);
    SmartDashboard.putNumber("Shooter/Hard Indexer Forward Power", HARD_INDEXER_FORWARD_POWER);
    SmartDashboard.putNumber("Shooter/Hard Reverse Seconds", HARD_INDEXER_REVERSE_SECONDS);
  }

  /**
   * Sets the launcher velocity in RPM using onboard PID.
   * @param targetRPM The target speed in RPM.
   */
  public void setLauncherVelocityRPM(double targetRPM) {
    if (targetRPM == 0) {
      leftFlywheel.set(0);
      rightFlywheel.set(0);
    } else {
      leftController.setSetpoint(targetRPM, ControlType.kVelocity);
      rightController.setSetpoint(targetRPM, ControlType.kVelocity);
    }
  }

  /**
   * Sets raw duty cycle power to the flywheels (used for intake).
   * @param power Power from -1.0 to 1.0.
   */
  public void setLauncherPower(double power) {
    leftFlywheel.set(power);
    rightFlywheel.set(power);
  }

  /**
   * Sets the indexer velocity in RPM.
   * @param targetRPM The target speed in RPM (positive for forward, negative for reverse).
   */
  public void setIndexerVelocityRPM(double targetRPM) {
    if (targetRPM == 0) {
      indexer.set(0);
    } else {
      indexerController.setSetpoint(targetRPM, ControlType.kVelocity);
    }
  }

  /**
   * Sets raw duty cycle power to the indexer (used for intake/unjam).
   * @param power Power from -1.0 to 1.0.
   */
  public void setIndexerPower(double power) {
    indexer.set(power);
  }

  /**
   * Returns the current flywheel velocity in RPM.
   */
  public double getVelocityRPM() {
    // Both motors should be at the same speed, return one.
    return rightFlywheel.getEncoder().getVelocity();
  }

  /**
   * Returns the current indexer velocity in RPM.
   */
  public double getLiveIndexerVelocity() {
    return indexer.getEncoder().getVelocity();
  }

  /**
   * Returns true if the launcher is within the given tolerance of the target RPM.
   */
  public boolean atSetpoint(double targetRPM, double toleranceRPM) {
    return Math.abs(getVelocityRPM() - targetRPM) < toleranceRPM;
  }

  /**
   * Stops all motors in the system.
   */
  public void stop() {
    leftFlywheel.set(0);
    rightFlywheel.set(0);
    indexer.set(0);
  }

  @Override
  public void periodic() {
    // Read tuning values from dashboard
    double p = SmartDashboard.getNumber("Shooter/P", lastP);
    double ff = SmartDashboard.getNumber("Shooter/FF", lastFF);

    // Update config and preferences if values changed on the dashboard
    if (p != lastP || ff != lastFF) {
      SparkMaxConfig tuningConfig = new SparkMaxConfig();
      tuningConfig.closedLoop.p(p);
      tuningConfig.closedLoop.feedForward.kV(ff);
      
      leftFlywheel.configure(tuningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      rightFlywheel.configure(tuningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      indexer.configure(tuningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      
      // Save to preferences for persistence
      Preferences.setDouble("Shooter/P", p);
      Preferences.setDouble("Shooter/FF", ff);
      
      lastP = p;
      lastFF = ff;
    }

    // Update status on dashboard
    SmartDashboard.putNumber("Shooter/Actual RPM", getVelocityRPM());
    SmartDashboard.putBoolean("Shooter/At Setpoint", atSetpoint(
        SmartDashboard.getNumber("Shooter/Target RPM", LAUNCHING_LAUNCHER_RPM), 
        SHOOTER_RPM_TOLERANCE));
  }

  /**
   * Returns the applied duty cycle (power) of the flywheels.
   */
  public double getFlywheelAppliedOutput() {
    return leftFlywheel.getAppliedOutput();
  }

  /**
   * Returns the applied duty cycle (power) of the indexer.
   */
  public double getIndexerAppliedOutput() {
    return indexer.getAppliedOutput();
  }
}
