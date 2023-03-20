package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SetDrivetrainMaxOutputCommand extends CommandBase {

    private double maxOutput;
    private DrivetrainSubsystem drivetrain;

    //Kinda wonky solution, does not set requirements for drivetrain. TODO Find better way?
    public SetDrivetrainMaxOutputCommand(double maxOutput, DrivetrainSubsystem drivetrain) {
        this.maxOutput = maxOutput;
        this.drivetrain = drivetrain;
    }
    
    public void execute() {
        drivetrain.setMaxOutput(maxOutput);
    }

    public void end(boolean interrupted) {
    }

    //Never ends until cancelled
    public boolean isFinished() {
        return true;
    }
}
