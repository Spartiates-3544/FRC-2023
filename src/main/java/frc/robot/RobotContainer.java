// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.commands.SetDrivetrainMaxOutputCommand;
import frc.robot.commands.SetTurretPositionCommand;
import frc.robot.commands.TrackApriltagCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(Constants.DriveConstants.controllerPort);
  CommandJoystick controller2 = new CommandJoystick(Constants.DriveConstants.controller2Port);
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final ManipulatorSubsystem manipulator = new ManipulatorSubsystem();
  private Trajectory trajectory = new Trajectory();

  public RobotContainer() {
    configureBindings();

    drivetrain.setDefaultCommand(
      Commands.run(() -> drivetrain.arcadeDrive(-controller.getLeftY(), ( controller.getRightTriggerAxis() - controller.getLeftTriggerAxis() )), drivetrain)
    );

    //turret.setDefaultCommand(new TrackApriltagCommand(turret, 1));

    //arm.setDefaultCommand(Commands.run(() -> { arm.setStage1Pourcentage(controller.getLeftY() * 0.5); arm.setStage2Pourcentage(controller.getRightY() * 0.5); }, arm));
  }

  private void configureBindings() {
    //Cube
    controller2.button(6).onTrue(Commands.parallel(new SetArmPositionCommand(arm, 2400, 1250), new SetDrivetrainMaxOutputCommand(0.5, drivetrain)));
    
    //Low
    controller2.button(7).onTrue(Commands.parallel(new SetArmPositionCommand(arm, 370, 1800), new SetDrivetrainMaxOutputCommand(0.7, drivetrain)));

    //TODO Fix PID when stowed
    //Stowed
    controller2.button(3).onTrue(Commands.parallel(new SetArmPositionCommand(arm, 330, 2775), new SetDrivetrainMaxOutputCommand(1, drivetrain)));

    controller.x().whileTrue(Commands.startEnd(() -> manipulator.setPourcentage(-0.4), () -> manipulator.setPourcentage(0), manipulator));
    controller.y().whileTrue(Commands.startEnd(() -> manipulator.setPourcentage(0.4), () -> manipulator.setPourcentage(0), manipulator));

  }

  public Command getAutonomousCommand() {
    //return Commands.print("No autonomous command configured");

    //TODO Douteux
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/Path1.wpilib.json");
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory", ex.getStackTrace());
    }

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, drivetrain::getPose,
     new RamseteController(DriveConstants.AutonomousConstants.kRamseteB, DriveConstants.AutonomousConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveConstants.AutonomousConstants.ksVolts, DriveConstants.AutonomousConstants.kvVoltSecondsPerMeter, DriveConstants.AutonomousConstants.kaVoltSecondsSquaredPerMeter),
       DriveConstants.AutonomousConstants.kDriveKinematics,
        drivetrain::getWheelSpeeds,
         new PIDController(DriveConstants.AutonomousConstants.kPDriveVel, 0, 0),
          new PIDController(DriveConstants.AutonomousConstants.kPDriveVel, 0, 0),
           drivetrain::tankDriveVolts,
            drivetrain);

    drivetrain.resetOdometry(trajectory.getInitialPose());

    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }
}
