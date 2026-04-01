package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.utils.MacroRecorder;
import static frc.robot.Constants.OperatorConstants.*;

public class RecordMacro extends Command {
    private final CANDriveSubsystem driveSubsystem;
    private final CommandXboxController controller;
    private final MacroRecorder recorder;
    private final java.util.function.Supplier<String> nameSupplier;

    public RecordMacro(CANDriveSubsystem driveSubsystem, CommandXboxController controller, MacroRecorder recorder, java.util.function.Supplier<String> nameSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.controller = controller;
        this.recorder = recorder;
        this.nameSupplier = nameSupplier;
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

        // Record the speed and rotation (better to record the raw inputs)
        recorder.record(xSpeed, zRotation);
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
