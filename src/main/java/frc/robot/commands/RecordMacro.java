package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.utils.MacroRecorder;
import static frc.robot.Constants.OperatorConstants.*;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class RecordMacro extends Command {
    private final CANDriveSubsystem driveSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final ClimberSubsystem climberSubsystem;
    private final CommandXboxController controller;
    private final MacroRecorder recorder;
    private final java.util.function.Supplier<String> nameSupplier;

    public RecordMacro(CANDriveSubsystem driveSubsystem, 
                       ShooterSubsystem shooterSubsystem,
                       ClimberSubsystem climberSubsystem,
                       CommandXboxController controller, 
                       MacroRecorder recorder, 
                       java.util.function.Supplier<String> nameSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.climberSubsystem = climberSubsystem;
        this.controller = controller;
        this.recorder = recorder;
        this.nameSupplier = nameSupplier;
        
        // Only REQUIRE the drive subsystem to ensure we keep driving it.
        // We only OBSERVE the other subsystems.
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        String name = nameSupplier.get();
        recorder.startRecording(name != null ? name : "macro1");
        SmartDashboard.putBoolean("Macro/Recording", true);
    }

    @Override
    public void execute() {
        double xSpeed = -controller.getLeftY() * driveSubsystem.getDriveScaling();
        double zRotation = -controller.getRightX() * ROTATION_SCALING;
        
        // Drive the robot normally
        driveSubsystem.driveArcade(xSpeed, zRotation);

        // Sample everything
        double shooterPower = shooterSubsystem.getFlywheelAppliedOutput();
        double indexerPower = shooterSubsystem.getIndexerAppliedOutput();
        double climberPower = climberSubsystem.getAppliedOutput();

        // Record the speeds and powers
        recorder.record(xSpeed, zRotation, shooterPower, indexerPower, climberPower);
    }

    @Override
    public void end(boolean interrupted) {
        recorder.stopRecording();
        driveSubsystem.driveArcade(0, 0);
        SmartDashboard.putBoolean("Macro/Recording", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
