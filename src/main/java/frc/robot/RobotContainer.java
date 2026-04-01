// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.OperatorConstants.*;

import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.Drive;
import frc.robot.commands.Eject;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.TurnAround;
import frc.robot.commands.Unjam;
import frc.robot.commands.RecordMacro;
import frc.robot.commands.ReplayMacro;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.utils.MacroRecorder;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final MacroRecorder macroRecorder = new MacroRecorder();

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller, by default it is setup to use a single controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  // PathPlanner auto chooser - automatically populated from deploy/pathplanner/autos/
  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<String> macroSlotChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Register named commands for use in PathPlanner auto routines.
    // Add these to any auto in the PathPlanner GUI as action blocks.
    NamedCommands.registerCommand("Intake", new Intake(shooterSubsystem));
    NamedCommands.registerCommand("Launch", new LaunchSequence(shooterSubsystem));
    NamedCommands.registerCommand("Arm Up", 
        Commands.run(() -> climberSubsystem.setClimber(frc.robot.Constants.ClimbConstants.CLIMBER_MOTOR_UP_PERCENT), climberSubsystem)
            .until(() -> climberSubsystem.getPosition() >= SmartDashboard.getNumber("Climber/Upper Limit", 100.0))
            .finallyDo((interrupted) -> climberSubsystem.stop()));
    NamedCommands.registerCommand("Arm Down", 
        Commands.run(() -> climberSubsystem.setClimber(frc.robot.Constants.ClimbConstants.CLIMBER_MOTOR_DOWN_PERCENT), climberSubsystem)
            .until(() -> climberSubsystem.getPosition() <= SmartDashboard.getNumber("Climber/Retract Limit", 0.0))
            .finallyDo((interrupted) -> climberSubsystem.stop()));

    configureBindings();

    // Build the auto chooser from PathPlanner only if AutoBuilder was configured
    // successfully (requires pathplanner/settings.json to exist in deploy dir).
    if (AutoBuilder.isConfigured()) {
      autoChooser = AutoBuilder.buildAutoChooser();
    } else {
      autoChooser = new SendableChooser<>();
      System.err.println("[RobotContainer] AutoBuilder not configured — auto chooser will be empty. " +
          "Open PathPlanner and save your robot config, or check settings.json.");
    }
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // Setup Macro Slots
    macroSlotChooser.setDefaultOption("Macro Slot 1", "macro1");
    macroSlotChooser.addOption("Macro Slot 2", "macro2");
    macroSlotChooser.addOption("Macro Slot 3", "macro3");
    SmartDashboard.putData("Macro/Recording Slot", macroSlotChooser);

    // Manual additions to auto chooser for macro replay
    autoChooser.addOption("Replay Macro 1", new ReplayMacro(driveSubsystem, macroRecorder, "macro1"));
    autoChooser.addOption("Replay Macro 2", new ReplayMacro(driveSubsystem, macroRecorder, "macro2"));
    autoChooser.addOption("Replay Macro 3", new ReplayMacro(driveSubsystem, macroRecorder, "macro3"));
    
    // Add Recording status to SmartDashboard
    SmartDashboard.putBoolean("Macro/Recording", false);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // While the left bumper on operator controller is held, intake Fuel
    driverController.leftBumper().whileTrue(new Intake(shooterSubsystem));
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    driverController.rightBumper().whileTrue(new LaunchSequence(shooterSubsystem));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    driverController.a().whileTrue(
        new Unjam(shooterSubsystem).withTimeout(1.0).andThen(new Eject(shooterSubsystem)));

    // When X button is pressed, activate turbo boost for 1.5 seconds
    driverController.x().onTrue(Commands.runOnce(() -> driveSubsystem.activateTurbo()));

    // Record Macro with D-Pad Down: Start recording when pressed, stop when pressed again (toggle)
    // Or just hold it? Toggle is usually safer for long sequences.
    // driverController.povDown().toggleOnTrue(new RecordMacro(driveSubsystem, driverController, macroRecorder));
    
    // Map D-Pad Down to Toggle Recording (Press once to start, Press again to stop)
    // It records to the slot currently selected in "Macro/Recording Slot"
    driverController.povDown().toggleOnTrue(new RecordMacro(driveSubsystem, driverController, macroRecorder, () -> macroSlotChooser.getSelected()));
    // Note: RecordMacro is interrupted when any other drive command takes over (like teleop default)
    // But since RecordMacro has priority while povDown is used, we need a way to end it.
    // If I use toggleOnTrue it will stay active.
    
    // Alternatively, just hold the button to record:
    // driverController.povDown().whileTrue(new RecordMacro(...));

   // While the down arrow on the directional pad is held it will unclimb the robot
    // driverController.povDown().whileTrue(new ClimbDown(climberSubsystem));
    // While the up arrow on the directional pad is held it will cimb the robot
    // driverController.povUp().whileTrue(new ClimbUp(climberSubsystem));

    driverController.povUp().onTrue(
        Commands.defer(() -> new TurnAround(driveSubsystem), Set.of(driveSubsystem)));

    // Require both Back and Start buttons at the same time to home the climber
    driverController.back().and(driverController.start()).onTrue(
        Commands.runOnce(() -> climberSubsystem.homeMotor())
    );

    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));

    shooterSubsystem.setDefaultCommand(shooterSubsystem.run(() -> shooterSubsystem.stop()));

    climberSubsystem.setDefaultCommand(
        Commands.run(
            () -> {
              // Left trigger goes up (positive), right trigger goes down (negative)
              double power = driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis();
              climberSubsystem.setClimber(power);
            },
            climberSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public ShooterSubsystem getShooterSubsystem() {
    return shooterSubsystem;
  }
}
