package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.TurretSubsystem;

public class TrackApriltagCommand extends PIDCommand{
    private TurretSubsystem turret;

    public TrackApriltagCommand(TurretSubsystem turret, int pipeline) {
        super(
            new PIDController(TurretConstants.limelightTurretP, TurretConstants.limelightTurretI, TurretConstants.limelightTurretD),
            () -> LimelightHelpers.getTX("limelight"),
            0,
            output -> turret.setTurret(-output),
            turret
            ); 
        LimelightHelpers.setPipelineIndex("limelight", pipeline);
        this.turret = turret;
    }

    public void initialize() {
        SmartDashboard.putBoolean("Tracking tourelle", true);
    }

    public void end(boolean interrupted) {
        turret.setMagicSetpoint(0);
        SmartDashboard.putBoolean("Tracking tourelle", false);
    }
}
