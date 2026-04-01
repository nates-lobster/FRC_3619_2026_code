package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax climberMotor;

  // Default to an arbitrarily large number, you can adjust this value from SmartDashboard!
  private double upperLimitDefault = 100.0;

  /** Creates a new CANBallSubsystem. */
  public ClimberSubsystem() {
    // create brushless motors for each of the motors on the launcher mechanism
    climberMotor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushless);

    // create the configuration for the climb moter, set a current limit and apply
    // the config to the controller
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig.smartCurrentLimit(CLIMBER_MOTOR_CURRENT_LIMIT);
    climbConfig.idleMode(IdleMode.kBrake);
    climbConfig.inverted(true);
    climberMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize Preferences from Constants if they don't exist
    Preferences.initDouble("Climber/Upper Limit", upperLimitDefault);
    Preferences.initDouble("Climber/Retract Limit", 0.0);
    
    // Add fields in SmartDashboard to set the limits.
    SmartDashboard.putNumber("Climber/Upper Limit", Preferences.getDouble("Climber/Upper Limit", upperLimitDefault));
    SmartDashboard.putNumber("Climber/Retract Limit", Preferences.getDouble("Climber/Retract Limit", 0.0));
    SmartDashboard.setDefaultBoolean("Climber/Override Limits", false);
  }

  // A method to set the percentage of the climber
  public void setClimber(double power) {
    double currentPosition = getPosition();
    double upperLimit = SmartDashboard.getNumber("Climber/Upper Limit", upperLimitDefault);
    double retractLimit = SmartDashboard.getNumber("Climber/Retract Limit", 0.0);

    // Check for override switch
    boolean override = SmartDashboard.getBoolean("Climber/Override Limits", false);

    if (!override) {
      // If we're trying to move UP (positive power) and at/past the upper limit, stop.
      if (power > 0 && currentPosition >= upperLimit) {
        power = 0;
      }
      // If we're trying to move DOWN (negative power) and at/past the retract limit, stop.
      else if (power < 0 && currentPosition <= retractLimit) {
        power = 0;
      }
    }

    climberMotor.set(power);
  }

  // A method to get the current position of the climber
  public double getPosition() {
    return climberMotor.getEncoder().getPosition();
  }

  // A method to stop the climber
  public void stop() {
    climberMotor.set(0);
  }

  // A method to home the climber motor (reset encoder to 0)
  public void homeMotor() {
    climberMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // Save to preferences if changed from dashboard
    double upper = SmartDashboard.getNumber("Climber/Upper Limit", -11.1);
    double retract = SmartDashboard.getNumber("Climber/Retract Limit", -11.1);

    if (upper != -11.1 && upper != Preferences.getDouble("Climber/Upper Limit", upperLimitDefault)) {
      Preferences.setDouble("Climber/Upper Limit", upper);
    }
    if (retract != -11.1 && retract != Preferences.getDouble("Climber/Retract Limit", 0.0)) {
      Preferences.setDouble("Climber/Retract Limit", retract);
    }

    // Project the encoder position to SmartDashboard
    SmartDashboard.putNumber("Climber/Encoder Position", climberMotor.getEncoder().getPosition());
  }
}
