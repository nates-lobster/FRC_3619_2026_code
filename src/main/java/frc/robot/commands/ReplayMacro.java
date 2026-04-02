package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.utils.MacroRecorder;
import frc.robot.utils.MacroRecorder.Step;

public class ReplayMacro extends Command {
    private final CANDriveSubsystem driveSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final ClimberSubsystem climberSubsystem;
    private final MacroRecorder recorder;
    private final String macroName;
    private List<Step> steps;
    private int currentStep;

    public ReplayMacro(CANDriveSubsystem driveSubsystem, 
                       ShooterSubsystem shooterSubsystem,
                       ClimberSubsystem climberSubsystem,
                       MacroRecorder recorder, 
                       String macroName) {
        this.driveSubsystem = driveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.climberSubsystem = climberSubsystem;
        this.recorder = recorder;
        this.macroName = macroName;
        addRequirements(driveSubsystem, shooterSubsystem, climberSubsystem);
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
            
            // Replay all subsystems
            driveSubsystem.driveArcade(step.driveX, step.driveRot, false);
            shooterSubsystem.setLauncherPower(step.shooterPower);
            shooterSubsystem.setIndexerPower(step.indexerPower);
            climberSubsystem.setClimber(step.climberPower);
            
            currentStep++;
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.driveArcade(0, 0);
        shooterSubsystem.stop();
        climberSubsystem.stop();
        System.out.println("[Macro] Replay finished.");
    }

    @Override
    public boolean isFinished() {
        return currentStep >= steps.size();
    }
}
