package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.utils.MacroRecorder;
import frc.robot.utils.MacroRecorder.Step;

public class ReplayMacro extends Command {
    private final CANDriveSubsystem driveSubsystem;
    private final MacroRecorder recorder;
    private final String macroName;
    private List<Step> steps;
    private int currentStep;

    public ReplayMacro(CANDriveSubsystem driveSubsystem, MacroRecorder recorder, String macroName) {
        this.driveSubsystem = driveSubsystem;
        this.recorder = recorder;
        this.macroName = macroName;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        steps = recorder.loadFromFile(macroName);
        currentStep = 0;
        System.out.println("[Macro] Replay started for " + macroName + ". Loaded " + steps.size() + " steps.");
    }

    @Override
    public void execute() {
        if (currentStep < steps.size()) {
            Step step = steps.get(currentStep);
            
            // Replay using the recorded xSpeed and zRotation
            // Note: Since we recorded xSpeed and zRotation, we use driveArcade(xSpeed, zRotation, false/true?)
            // We'll use false to avoid re-squaring if it was already squared.
            driveSubsystem.driveArcade(step.left, step.right, false);
            
            currentStep++;
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.driveArcade(0, 0);
        System.out.println("[Macro] Replay finished.");
    }

    @Override
    public boolean isFinished() {
        return currentStep >= steps.size();
    }
}
