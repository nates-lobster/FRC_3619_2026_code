package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    // Add fields in SmartDashboard to set the limits.
    SmartDashboard.setDefaultNumber("Climber/Upper Limit", upperLimitDefault);
    SmartDashboard.setDefaultNumber("Climber/Retract Limit", 0.0);
  }

  // A method to set the percentage of the climber
  public void setClimber(double power) {
    double currentPosition = getPosition();
    double upperLimit = SmartDashboard.getNumber("Climber/Upper Limit", upperLimitDefault);
    double retractLimit = SmartDashboard.getNumber("Climber/Retract Limit", 0.0);

    // If we're trying to move UP (positive power) and at/past the upper limit, stop.
    if (power > 0 && currentPosition >= upperLimit) {
      power = 0;
    }
    // If we're trying to move DOWN (negative power) and at/past the retract limit, stop.
    else if (power < 0 && currentPosition <= retractLimit) {
      power = 0;
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
    // Project the encoder position to SmartDashboard
    SmartDashboard.putNumber("Climber/Encoder Position", climberMotor.getEncoder().getPosition());
  }
}
