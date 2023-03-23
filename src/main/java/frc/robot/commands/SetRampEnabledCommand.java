package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SetRampEnabledCommand extends CommandBase {
    private boolean enabled;
    private DrivetrainSubsystem drivetrain;

    public SetRampEnabledCommand(DrivetrainSubsystem drivetrain, boolean enabled) {
        this.drivetrain = drivetrain;
        this.enabled = enabled;
    }

    public void execute() {
        drivetrain.enableRamp(enabled);
    }

    public boolean isFinished() {
        return true;
    }

}
