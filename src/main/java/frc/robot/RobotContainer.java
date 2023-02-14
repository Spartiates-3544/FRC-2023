// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants.Armpositions;
import frc.robot.Constants.TurretConstants.Turretpositions;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.commands.SetTurretPositionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(Constants.DriveConstants.controllerPort);
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();

  public RobotContainer() {
    configureBindings();

    drivetrain.setDefaultCommand(
      Commands.run(() -> drivetrain.arcadeDrive(controller.getLeftX() * .5, controller.getLeftY() * .5), drivetrain)
    );

    //Calibrate turret on startup (ofc when the robot is initially enabled)
    CommandScheduler.getInstance().schedule(new FunctionalCommand(
      null,
     () -> turret.setTurret(-0.2),
      interrupted -> {turret.setTurret(0); turret.resetEncoder();},
       () -> turret.getCalibrationSwitch(), turret)
       );
  }

  private void configureBindings() {
    //Change arm position
    controller.b().onTrue(new SequentialCommandGroup(
      new SetArmPositionCommand(arm, Armpositions.STOWED), 
      new SetTurretPositionCommand(turret, Turretpositions.BACK), 
      new SetArmPositionCommand(arm, Armpositions.HIGH)
      ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
