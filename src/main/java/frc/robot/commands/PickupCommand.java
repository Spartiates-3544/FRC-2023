package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;

public class PickupCommand extends CommandBase{

    private ManipulatorSubsystem manipulator;
    private boolean cube;
    private int counter = 0;

    public PickupCommand(ManipulatorSubsystem manipulator, boolean cube) {
        this.manipulator = manipulator;
        this.cube = cube;
    }

    public void execute() {
        if (cube) {
            manipulator.setPourcentage(-0.4);
        }
        manipulator.setPourcentage(0.4);

        
        if (manipulator.getOutputCurrent() >= 20) {
            counter++;
        }

        if (manipulator.getOutputCurrent() < 20) {
            counter--;
        }

        SmartDashboard.putNumber("Counter", counter);
    }

    public void end(boolean interrupted) {
        manipulator.setPourcentage(0);
        counter = 0;
    }

    public boolean isFinished() {
        return counter > 10;

    }

}
