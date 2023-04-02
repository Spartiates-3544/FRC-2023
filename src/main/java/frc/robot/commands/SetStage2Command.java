package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.Utilities.Utilities;
import frc.robot.subsystems.ArmSubsystem;

public class SetStage2Command extends CommandBase {
    private ArmSubsystem arm;
    private double stage2Pos;

    public SetStage2Command(ArmSubsystem arm, double stage2Pos) {
        this.arm = arm;
        this.stage2Pos = stage2Pos;
        addRequirements(arm);
    }

    public void execute() {
        arm.setStage2Pos(stage2Pos);
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
        return Utilities.inRange(stage2Pos - 50, stage2Pos + 50, arm.getStage2Encoder());
    }
}

