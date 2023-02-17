package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

public class SetTurretPositionCommand extends CommandBase {
    private TurretSubsystem turret;
    private TurretConstants.TurretPos position;

    public SetTurretPositionCommand(TurretSubsystem turret, TurretConstants.TurretPos position) {
        this.turret = turret;
        this.position = position;
        addRequirements(this.turret);
    }

    public void initialise() {
        switch (position) {
            case FRONT:
                turret.setMagicSetpoint(TurretConstants.TurretPositions.front);
                break;
        
            case BACK:
                turret.setMagicSetpoint(TurretConstants.TurretPositions.back);
                break;

            default:
                turret.setMagicSetpoint(TurretConstants.TurretPositions.front);
                break;
        }
    }

    public boolean isFinished() {
        return turret.atSetpoint();
    }
}
