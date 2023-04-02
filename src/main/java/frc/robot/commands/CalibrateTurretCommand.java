package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class CalibrateTurretCommand extends CommandBase {
    private TurretSubsystem turret;

    public CalibrateTurretCommand(TurretSubsystem turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    public void execute() {
        if (turret.getEncoder() > 0) {
            turret.setTurret(-0.35);
        }

        if (turret.getEncoder() < 0) {
            turret.setTurret(0.35);
        }
    }

    public void end(boolean interrupted) {
        turret.setTurret(0);
        turret.resetEncoder();
    }


    public boolean isFinished() {
        return turret.getCalibrationSwitch();
    }

}
