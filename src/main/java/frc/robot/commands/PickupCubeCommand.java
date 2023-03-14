package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;

public class PickupCubeCommand extends CommandBase{

    ManipulatorSubsystem manipulator;
    double lastCurrent;

    public PickupCubeCommand(ManipulatorSubsystem manipulator) {
        this.manipulator = manipulator;
        lastCurrent = manipulator.getOutputCurrent();
    }

    public void execute() {
        manipulator.setPourcentage(0.4);
        lastCurrent = manipulator.getOutputCurrent();
    }

    public void end(boolean interrupted) {
        manipulator.setPourcentage(0);
    }

    public boolean isFinished() {
        return false; //TODO add end condition (detect when there's a game piece)
    }

}
