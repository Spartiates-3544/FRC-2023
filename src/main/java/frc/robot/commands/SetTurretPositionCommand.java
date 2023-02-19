package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.Utilities.Utilities;
import frc.robot.subsystems.TurretSubsystem;

public class SetTurretPositionCommand extends CommandBase {
    private TurretSubsystem turret;
    private double position;

    public SetTurretPositionCommand(TurretSubsystem turret, double position) {
        this.turret = turret;
        this.position = position;
        addRequirements(this.turret);
    }

    public void initialise() {
    }

    public void execute() {
        turret.setMagicSetpoint(position);
    }

    public boolean isFinished() {
        return Utilities.inRange(position - 50, position + 50, turret.getEncoder());
    }
}
