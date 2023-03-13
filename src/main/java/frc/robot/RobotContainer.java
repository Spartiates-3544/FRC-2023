// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SetArmPositionCommand;
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
    controller2.button(6).onTrue(new SetArmPositionCommand(arm, 2400, 1250));

    //controller2.button(6).onTrue(Commands.parallel(new SetArmPositionCommand(arm, 2400, 1250), Commands.runEnd(() -> drivetrain.setMaxOutput(0.6), drivetrain.setMaxOutput(1),)));
    
    //Low
    controller2.button(7).onTrue(new SetArmPositionCommand(arm, 370, 1800));
    //Stowed
    controller2.button(3).onTrue(new SetArmPositionCommand(arm, 330, 2775));

    controller.x().whileTrue(Commands.startEnd(() -> manipulator.setPourcentage(-0.4), () -> manipulator.setPourcentage(0), manipulator));
    controller.y().whileTrue(Commands.startEnd(() -> manipulator.setPourcentage(0.4), () -> manipulator.setPourcentage(0), manipulator));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
