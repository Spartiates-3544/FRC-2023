package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

public class SetArmPositionCommand extends CommandBase {
    private TurretSubsystem turret;
    private TurretConstants.Armpositions position;

    public SetArmPositionCommand(TurretSubsystem turret, TurretConstants.Armpositions position) {
        this.turret = turret;
        this.position = position;
        addRequirements(this.turret);
    }

    public void initialise() {
        switch (position) {
            case FRONT:
                turret.setMagicSetpoint(TurretConstants.ArmPositions.front);
                break;
        
            case BACK:
                turret.setMagicSetpoint(TurretConstants.ArmPositions.back);
                break;

            default:
                turret.setMagicSetpoint(TurretConstants.ArmPositions.front);
                break;
        }
    }
}
