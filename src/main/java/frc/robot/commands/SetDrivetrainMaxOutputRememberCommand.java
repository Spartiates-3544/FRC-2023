package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SetDrivetrainMaxOutputRememberCommand extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private double maxOutput;
    private double prevMaxOutput = 1;

    public SetDrivetrainMaxOutputRememberCommand(DrivetrainSubsystem drivetrain, double maxOutput) {
        this.drivetrain = drivetrain;
        this.maxOutput = maxOutput;
    }

    public void initialize() {
        prevMaxOutput = drivetrain.getMaxOutput();
    }

    public void execute() {
        drivetrain.setMaxOutput(maxOutput);
    }

    public void end(boolean interrupted) {
        drivetrain.setMaxOutput(prevMaxOutput);
    }

    public boolean isFinished() {
        return false;
    }
}
