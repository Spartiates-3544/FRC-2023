package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmPositionCommand extends CommandBase {
    private ArmSubsystem arm;
    private ArmConstants.Armpositions position;

    public SetArmPositionCommand(ArmSubsystem arm, ArmConstants.Armpositions position) {
        this.arm = arm;
        this.position = position;
        addRequirements(this.arm);
    }

    public void initialise() {
        switch (position) {
            case STOWED:
                arm.setStage1Pos(ArmConstants.ArmPositions.stowed1);
                arm.setStage2Pos(ArmConstants.ArmPositions.stowed2);
                break;
        
            case GROUND:
                arm.setStage1Pos(ArmConstants.ArmPositions.ground1);
                arm.setStage2Pos(ArmConstants.ArmPositions.ground2);
                break;

            case HIGH:
                arm.setStage1Pos(ArmConstants.ArmPositions.high1);
                arm.setStage2Pos(ArmConstants.ArmPositions.high2);
                break;

            default:
                arm.setStage1Pos(ArmConstants.ArmPositions.stowed1);
                arm.setStage2Pos(ArmConstants.ArmPositions.stowed2);
                break;
        }
    }

    public boolean isFinished() {
        return arm.stage1AtSetpoint() && arm.stage2AtSetpoint();
    }
}
