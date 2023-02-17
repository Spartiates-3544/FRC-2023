// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.ArmConstants.Armpositions;
import frc.robot.Constants.TurretConstants.TurretPos;
import frc.robot.commands.CalibrateTurretCommand;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.commands.SetTurretPositionCommand;
import frc.robot.commands.TrackApriltagCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(Constants.DriveConstants.controllerPort);
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final Limelight limeight = new Limelight();

  public RobotContainer() {
    configureBindings();

    drivetrain.setDefaultCommand(
      Commands.run(() -> drivetrain.arcadeDrive(controller.getLeftY() * .5, controller.getRightX() * .5), drivetrain)
    );

    turret.setDefaultCommand(new TrackApriltagCommand(turret, limeight, TurretConstants.Apriltags.REDRIGHT1));
  }

  private void configureBindings() {
    
    //controller.a().onTrue(new SetTurretPositionCommand(turret, Constants.TurretConstants.TurretPos.BACK));
    controller.a().onTrue(Commands.runOnce(() -> turret.setMagicSetpoint(231000), turret));
    controller.x().onTrue(Commands.runOnce(() -> turret.setMagicSetpoint(0), turret));
    //Change arm position
    /*controller.b().onTrue(new SequentialCommandGroup(
      new SetArmPositionCommand(arm, Armpositions.STOWED), 
      new SetTurretPositionCommand(turret, Turretpositions.BACK), 
      new SetArmPositionCommand(arm, Armpositions.HIGH)
      ));
      */
    controller.y().onTrue(new CalibrateTurretCommand(turret));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
