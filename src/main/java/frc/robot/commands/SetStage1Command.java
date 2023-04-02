package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.Utilities.Utilities;
import frc.robot.subsystems.ArmSubsystem;

public class SetStage1Command extends CommandBase {
    private ArmSubsystem arm;
    private double stage1Pos;

    public SetStage1Command(ArmSubsystem arm, double stage1Pos) {
        this.arm = arm;
        this.stage1Pos = stage1Pos;
    }

    public void execute() {
        arm.setStage1Pos(stage1Pos);
    }

    public void end(boolean interrupted) {
        /*
        if (interrupted) {
            arm.setStage1Pourcentage(0);
            arm.setStage2Pourcentage(0);
        }
        */
    }

    public boolean isFinished() {
        //return arm.stage1AtSetpoint() && arm.stage2AtSetpoint();
        return Utilities.inRange(stage1Pos - 50, stage1Pos + 50, arm.getStage1Encoder());
    }
}

