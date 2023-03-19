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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.commands.SetDrivetrainMaxOutputCommand;
import frc.robot.commands.TrackApriltagCommand;
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
  private Trajectory trajectory = new Trajectory();

  //True for cone, False for cube
  private boolean mode = true;

  public RobotContainer() {
    configureBindings();

    drivetrain.setDefaultCommand(
      Commands.run(() -> drivetrain.arcadeDrive(-controller.getLeftY(), ( controller.getRawAxis(4) - controller.getRawAxis(5) ) ), drivetrain)
    );

    CommandScheduler.getInstance().onCommandInitialize(command -> System.out.print("Command scheduled: " + command.getName()));
    
  }

  private void configureBindings() {
    //Toggle game piece mode
    controller2.button(11).onTrue(Commands.run(() -> mode = !mode));
    
    
    
    
    
    
    
    
    
    
    
    
    
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

    //TODO Douteux
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("Unnamed.wpilib.json");
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
