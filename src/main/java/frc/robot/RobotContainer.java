// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.PickupCommand;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.commands.SetDrivetrainMaxOutputCommand;
import frc.robot.commands.SetRampEnabledCommand;
import frc.robot.commands.TrackApriltagCommand;
import frc.robot.commands.commandgroups.HPShelfConeCommandGroup;
import frc.robot.commands.commandgroups.HPShelfCubeCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(Constants.DriveConstants.controllerPort);
  private final CommandJoystick controller2 = new CommandJoystick(Constants.DriveConstants.controller2Port);
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final ManipulatorSubsystem manipulator = new ManipulatorSubsystem();
  private LEDSubsystem leds = new LEDSubsystem();

  //True for cone, False for cube
  private boolean mode = true;

  private Trigger cube = new Trigger(() -> mode == false);
  private Trigger cone = new Trigger(() -> mode == true);

  public RobotContainer() {
    configureBindings();

    drivetrain.setDefaultCommand(
      Commands.run(() -> drivetrain.arcadeDrive(-controller.getLeftY(), ( controller.getRightTriggerAxis() - controller.getLeftTriggerAxis() ) * 0.8 ), drivetrain)
    );

    //arm.setDefaultCommand(Commands.run(() -> {arm.setStage1Pourcentage(controller.getLeftY()); arm.setStage2Pourcentage(controller.getRightY());}, arm));
    CommandScheduler.getInstance().onCommandInitialize(command -> System.out.print("Command scheduled: " + command.getName()));
    
  }

  private void configureBindings() {
    //Toggle game piece mode
    controller2.button(3).onTrue(Commands.runOnce(() -> mode = !mode));

    cube.whileTrue(Commands.runOnce(() -> leds.setRGB(255, 200, 0), leds));
    cone.whileTrue(Commands.runOnce(() -> leds.setRGB(130, 0, 255), leds));
    
    //Intake
    controller2.button(2).and(() -> mode == false).whileTrue(Commands.runEnd(() -> manipulator.setPourcentage(0.4), () -> manipulator.setPourcentage(0), manipulator));
    controller2.button(2).and(() -> mode == true).whileTrue(Commands.runEnd(() -> manipulator.setPourcentage(-0.4), () -> manipulator.setPourcentage(0), manipulator));

    //Outtake
    controller2.button(1).and(() -> mode == false).whileTrue(Commands.runEnd(() -> manipulator.setPourcentage(-0.4), () -> manipulator.setPourcentage(0), manipulator));
    controller2.button(1).and(() -> mode == true).whileTrue(Commands.runEnd(() -> manipulator.setPourcentage(0.4), () -> manipulator.setPourcentage(0), manipulator));
    
    /* Arm positions */
    //Low 
    controller2.button(11).and(() -> mode == false).onTrue(Commands.sequence(new SetDrivetrainMaxOutputCommand(0.8, drivetrain), new SetRampEnabledCommand(drivetrain, true), new SetArmPositionCommand(arm, 200, 1910)));
    controller2.button(11).and(() -> mode == true).onTrue(Commands.sequence(new SetDrivetrainMaxOutputCommand(0.8, drivetrain), new SetRampEnabledCommand(drivetrain, true), new SetArmPositionCommand(arm, 200, 1930)));
    //Mid
    controller2.button(9).and(() -> mode == false).onTrue(Commands.sequence(new SetDrivetrainMaxOutputCommand(0.7, drivetrain), new SetRampEnabledCommand(drivetrain, false), new SetArmPositionCommand(arm, 1246, 1818), Commands.waitSeconds(2), Commands.run(() -> manipulator.setPourcentage(-0.4), manipulator).withTimeout(1), Commands.runOnce(() -> manipulator.setPourcentage(0), manipulator)));
    controller2.button(9).and(() -> mode == true).onTrue(Commands.sequence(new SetDrivetrainMaxOutputCommand(0.7, drivetrain), new SetRampEnabledCommand(drivetrain, false), new SetArmPositionCommand(arm, 1246, 1818), Commands.waitSeconds(2), Commands.run(() -> manipulator.setPourcentage(0.4), manipulator).withTimeout(1), Commands.runOnce(() -> manipulator.setPourcentage(0), manipulator)));

    //High
    controller2.button(7).and(() -> mode == false).onTrue(Commands.sequence(new SetDrivetrainMaxOutputCommand(0.4, drivetrain), new SetRampEnabledCommand(drivetrain, false), new SetArmPositionCommand(arm, 2513, 1451), Commands.waitSeconds(2), Commands.run(() -> manipulator.setPourcentage(-0.4), manipulator).withTimeout(1), Commands.runOnce(() -> manipulator.setPourcentage(0), manipulator)));
    controller2.button(7).and(() -> mode == true).onTrue(Commands.sequence(new SetDrivetrainMaxOutputCommand(0.4, drivetrain), new SetRampEnabledCommand(drivetrain, false), new SetArmPositionCommand(arm, 2325, 1469), Commands.waitSeconds(2), Commands.run(() -> manipulator.setPourcentage(0.4), manipulator).withTimeout(1), Commands.runOnce(() -> manipulator.setPourcentage(0), manipulator)));


    //HP Shelf
    //controller2.button(8).and(() -> mode == false).onTrue(Commands.sequence(new SetDrivetrainMaxOutputCommand(0.4, drivetrain), new SetRampEnabledCommand(drivetrain, false), new SetArmPositionCommand(arm, 3016, 1099)));
    //controller2.button(8).and(() -> mode == true).onTrue(Commands.sequence(new SetDrivetrainMaxOutputCommand(0.4, drivetrain), new SetRampEnabledCommand(drivetrain, false), new SetArmPositionCommand(arm, 3016, 1099)));

    controller2.button(8).and(() -> mode == false).onTrue(new HPShelfCubeCommandGroup(drivetrain, arm, turret, manipulator));
    controller2.button(8).and(() -> mode == true).onTrue(new HPShelfConeCommandGroup(drivetrain, arm, turret, manipulator));

    //Stowed
    controller2.button(12).onTrue(Commands.sequence(new SetArmPositionCommand(arm, 200, 2868), new SetRampEnabledCommand(drivetrain, true), new SetDrivetrainMaxOutputCommand(1, drivetrain)));
    
    //Cone pickup (does not work)
    //controller2.button(10).onTrue(new SetArmPositionCommand(arm, 344, 1587));


    //controller2.button(4).onTrue(new PickupCommand(manipulator, false));
    controller2.button(4).toggleOnTrue(new TrackApriltagCommand(turret, 0));
    


    //Feedforward calibration poses
    //controller2.button(11).and(() -> mode == false).onTrue(new SetArmPositionCommand(arm, 2343, 1060));
    //controller2.button(11).and(() -> mode == true).onTrue(new SetArmPositionCommand(arm, 2343, 2073));

    
    /*
    //High
    controller2.button(6).onTrue(Commands.parallel(new SetArmPositionCommand(arm, 2400, 1350), new SetDrivetrainMaxOutputCommand(0.3, drivetrain)));
    //Low
    controller2.button(7).onTrue(Commands.parallel(new SetArmPositionCommand(arm, 370, 1850), new SetDrivetrainMaxOutputCommand(0.5, drivetrain)));
    //Fix PID when stowed (Fixed itself somehow??)
    //Stowed
    controller2.button(3).onTrue(Commands.parallel(new SetArmPositionCommand(arm, 330, 2775), new SetDrivetrainMaxOutputCommand(0.5, drivetrain)));


    //controller2.button(4).whileTrue(Commands.startEnd(() -> {manipulator.setPourcentage(-0.4); leds.setRGB(130, 0, 255);}, () -> manipulator.setPourcentage(0), manipulator));
    controller2.button(4).whileTrue(Commands.repeatingSequence(Commands.runOnce(() -> leds.setRGB(130, 0, 255), leds) , Commands.waitSeconds(0.1), Commands.runOnce(() -> leds.setRGB(0, 0, 0), leds) , Commands.waitSeconds(0.1)));

    //controller2.button(5).whileTrue(Commands.startEnd(() -> {manipulator.setPourcentage(0.4); leds.setRGB(255, 200, 0);}, () -> manipulator.setPourcentage(0), manipulator));
    controller2.button(5).whileTrue(Commands.repeatingSequence(Commands.runOnce(() -> leds.setRGB(255, 200, 0), leds) , Commands.waitSeconds(0.1), Commands.runOnce(() -> leds.setRGB(0, 0, 0), leds) , Commands.waitSeconds(0.1)));

    controller.rightBumper().toggleOnTrue(new TrackApriltagCommand(turret, 0));
    //,controller.leftBumper().toggleOnTrue(Commands.repeatingSequence(new ToggleLEDsCommand(leds), new WaitCommand(0.5)));
    */

  }

  public Command getAutonomousCommand() {
    //return Commands.print("No autonomous command configured");

    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            DriveConstants.AutonomousConstants.ksVolts,
            DriveConstants.AutonomousConstants.kvVoltSecondsPerMeter,
            DriveConstants.AutonomousConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.AutonomousConstants.kDriveKinematics,
        10);

            // Create config for trajectory
    TrajectoryConfig config =
      new TrajectoryConfig(
            DriveConstants.AutonomousConstants.kMaxSpeedMetersPerSecond,
            DriveConstants.AutonomousConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.AutonomousConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
  Trajectory exampleTrajectory =
      TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(3, 0, new Rotation2d(0)),
          // Pass config
          config);

    /*/
    //TODO Douteux
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("Calibration.wpilib.json");
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory", ex.getStackTrace());
    }
    */

    RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, drivetrain::getPose,
     new RamseteController(DriveConstants.AutonomousConstants.kRamseteB, DriveConstants.AutonomousConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveConstants.AutonomousConstants.ksVolts, DriveConstants.AutonomousConstants.kvVoltSecondsPerMeter, DriveConstants.AutonomousConstants.kaVoltSecondsSquaredPerMeter),
     //new SimpleMotorFeedforward(DriveConstants.AutonomousConstants.ksVolts, DriveConstants.AutonomousConstants.kvVoltSecondsPerMeter, DriveConstants.AutonomousConstants.kaVoltSecondsSquaredPerMeter),
       DriveConstants.AutonomousConstants.kDriveKinematics,
        drivetrain::getWheelSpeeds,
         new PIDController(DriveConstants.AutonomousConstants.kPDriveVel, 0, 0),
          new PIDController(DriveConstants.AutonomousConstants.kPDriveVel, 0, 0),
           drivetrain::tankDriveVolts,
            drivetrain);

      //RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, drivetrain::getPose, new RamseteController(DriveConstants.AutonomousConstants.kRamseteB, DriveConstants.AutonomousConstants.kRamseteZeta), DriveConstants.AutonomousConstants.kDriveKinematics, drivetrain::followVelocites, drivetrain);

    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }
}
