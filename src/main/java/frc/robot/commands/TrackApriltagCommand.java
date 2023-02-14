package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.TurretSubsystem;

public class TrackApriltagCommand extends PIDCommand{


    public TrackApriltagCommand(TurretSubsystem turret, Limelight limelight, TurretConstants.Apriltags selectedTag) {
        super(
            new PIDController(TurretConstants.turretP, TurretConstants.turretI, TurretConstants.turretD),
            limelight::getTx,
            0,
            output -> turret.setTurret(output),
            turret
            ); 
        
        limelight.setPipeline(selectedTag);
    }
}
