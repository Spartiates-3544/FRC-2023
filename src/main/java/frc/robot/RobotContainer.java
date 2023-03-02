// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.commands.SetTurretPositionCommand;
import frc.robot.commands.TrackApriltagCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(Constants.DriveConstants.controllerPort);
  CommandJoystick controller2 = new CommandJoystick(Constants.DriveConstants.controller2Port);
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();

  public RobotContainer() {
    configureBindings();

    drivetrain.setDefaultCommand(
      Commands.run(() -> drivetrain.arcadeDrive(controller.getLeftY() * .5, controller.getRightX() * .5), drivetrain)
    );

    turret.setDefaultCommand(new TrackApriltagCommand(turret, TurretConstants.Apriltags.REDRIGHT1));
  }

  private void configureBindings() {
    
    //controller.a().onTrue(new SetTurretPositionCommand(turret, Constants.TurretConstants.TurretPos.BACK));
    controller.a().onTrue(Commands.runOnce(() -> turret.setMagicSetpoint(231000), turret));

    //Go to front, Mid
    controller2.button(7).onTrue(new SequentialCommandGroup(new SetArmPositionCommand(arm, 0, 0), new SetTurretPositionCommand(turret, 0), new SetArmPositionCommand(arm, 82386, -73897)));

    //Reset encoders
    controller.y().onTrue(Commands.runOnce(() -> {arm.resetStage1Encoder(); arm.resetStage2Encoder();}, arm));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
