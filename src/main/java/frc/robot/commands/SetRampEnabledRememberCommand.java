package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SetRampEnabledRememberCommand extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private boolean enabled;
    private boolean prevEnabled = false;

    public SetRampEnabledRememberCommand(DrivetrainSubsystem drivetrain, boolean enabled) {
        this.drivetrain = drivetrain;
        this.enabled = enabled;
    }

    public void initialize() {
        prevEnabled = drivetrain.getRampEnabled();
    }

    public void execute() {
        drivetrain.enableRamp(enabled);
    }

    public void end(boolean interrupted) {
        drivetrain.enableRamp(prevEnabled);
    }

    public boolean isFinished() {
        return false;
    }
}
