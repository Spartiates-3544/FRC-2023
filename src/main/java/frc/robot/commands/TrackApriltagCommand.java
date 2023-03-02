package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.TurretSubsystem;

public class TrackApriltagCommand extends PIDCommand{

    public TrackApriltagCommand(TurretSubsystem turret, TurretConstants.Apriltags selectedTag) {
        super(
            new PIDController(TurretConstants.limelightTurretP, TurretConstants.limelightTurretI, TurretConstants.limelightTurretD),
            () -> LimelightHelpers.getTX("limelight"),
            0,
            output -> turret.setTurret(-output),
            turret
            ); 
        //limelight.setPipeline(selectedTag);
    }
}
