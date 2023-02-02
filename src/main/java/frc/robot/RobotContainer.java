// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(Constants.DriveConstants.controllerPort);
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();

  public RobotContainer() {
    configureBindings();

    drivetrain.setDefaultCommand(
      Commands.run(() -> drivetrain.arcadeDrive(controller.getLeftX(), controller.getLeftY()), drivetrain)
    );
  }

  private void configureBindings() {
    controller.a().onTrue(arm.setArmGrabConeHP());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
