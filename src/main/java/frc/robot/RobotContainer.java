// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CalibrateTurretCommand;
import frc.robot.commands.PickupCommand;
import frc.robot.commands.RunPathCommand;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.commands.SetDrivetrainMaxOutputCommand;
import frc.robot.commands.SetDrivetrainMaxOutputRememberCommand;
import frc.robot.commands.SetRampEnabledCommand;
import frc.robot.commands.SetRampEnabledRememberCommand;
import frc.robot.commands.SetTurretPositionCommand;
import frc.robot.commands.TrackApriltagCommand;
import frc.robot.commands.AutoCommands.ConeHautSortirCommand;
import frc.robot.commands.AutoCommands.ConeHighThenCubeMidCommand;
import frc.robot.commands.AutoCommands.CubeHautSortirCommand;
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
  public final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final ManipulatorSubsystem manipulator = new ManipulatorSubsystem();
  private LEDSubsystem leds = new LEDSubsystem();

  //True for cone, False for cube
  private boolean mode = true;

  private boolean hasGameObject = false;

  private Trigger cube = new Trigger(() -> mode == false);
  private Trigger cone = new Trigger(() -> mode == true);

  private Trigger gameObject = new Trigger(() -> hasGameObject);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private HttpCamera limelightFeed;

  String[] path1 = {"/home/lvuser/deploy/paths/BalanceChargeStation.json"};
  String[] path2 = {"/home/lvuser/deploy/paths/AutonomeReculer.json"};

  public RobotContainer() {
    //Autonomous routines
    autoChooser.setDefaultOption("Aucun", Commands.run(() -> drivetrain.arcadeDrive(0, 0)));
    autoChooser.addOption("Cone to out of zone", new ConeHautSortirCommand(drivetrain, arm, turret, manipulator));
    autoChooser.addOption("Cube to out of zone", new CubeHautSortirCommand(drivetrain, arm, turret, manipulator));
    autoChooser.addOption("Straight to Charge station", new RunPathCommand(drivetrain, path1).withTimeout(4.3).andThen(Commands.runOnce(() -> drivetrain.setNeutralMode(NeutralMode.Brake), drivetrain)));
    autoChooser.addOption("Straight out of zone", new RunPathCommand(drivetrain, path2));
    autoChooser.addOption("2 piece", new ConeHighThenCubeMidCommand(drivetrain, arm, turret, manipulator));
    Shuffleboard.getTab("Autonomous").add(autoChooser).withPosition(6, 2);

    limelightFeed = new HttpCamera("limelight", "http://10.35.44.11:5800/stream.mjpg", HttpCameraKind.kMJPGStreamer);
    Shuffleboard.getTab("Autonomous").add(limelightFeed).withPosition(0, 0).withSize(6, 5);

    Shuffleboard.getTab("Teleop").add(limelightFeed).withPosition(0, 0).withSize(6, 5);
    Shuffleboard.getTab("Teleop").addBoolean("Mode", () -> mode).withPosition(7, 0).withSize(2, 2).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color when True", "#ffb100", "Color when False", "810081"));

    configureBindings();

    
    drivetrain.setDefaultCommand(
      Commands.run(() -> drivetrain.arcadeDrive(-controller.getLeftY(), ( controller.getRightTriggerAxis() - controller.getLeftTriggerAxis() ) * 0.8 ), drivetrain)
    );
    
    //turret.setDefaultCommand(Commands.run(() -> turret.setTurret(controller.getRightX()), turret));

    //arm.setDefaultCommand(Commands.run(() -> {arm.setStage1Pourcentage(controller.getLeftY()); arm.setStage2Pourcentage(controller.getRightY());}, arm));
    CommandScheduler.getInstance().onCommandInitialize(command -> System.out.print("Command scheduled: " + command.getName()));
    
  }

  private void configureBindings() {
    
    
    //Toggle game piece mode
    controller2.button(3).onTrue(Commands.runOnce(() -> mode = !mode));

    cone.onTrue(Commands.runOnce(() -> leds.setRGB(255, 150, 5), leds).andThen(Commands.runOnce(() -> hasGameObject = false)));
    cube.onTrue(Commands.runOnce(() -> leds.setRGB(100, 0, 255), leds).andThen(Commands.runOnce(() -> hasGameObject = false)));

    gameObject.and(() -> mode == false).whileFalse(Commands.repeatingSequence(Commands.runOnce(() -> leds.setRGB(130, 0, 255), leds), Commands.waitSeconds(0.5), Commands.runOnce(() -> leds.setRGB(0, 0, 0), leds), Commands.waitSeconds(0.5)));
    gameObject.and(() -> mode == true).whileFalse(Commands.repeatingSequence(Commands.runOnce(() -> leds.setRGB(255, 200, 0), leds), Commands.waitSeconds(0.5), Commands.runOnce(() -> leds.setRGB(0, 0, 0), leds), Commands.waitSeconds(0.5)));

    //Turret positions (Increments of 90 degrees)
    //controller2.povUp().onTrue(new SetTurretPositionCommand(turret, 0));
    controller2.povUp().onTrue(new CalibrateTurretCommand(turret));
    controller2.povDown().onTrue(new SetTurretPositionCommand(turret, 229028));
    controller2.povRight().onTrue(new SetTurretPositionCommand(turret, 112078));
    controller2.povLeft().onTrue(new SetTurretPositionCommand(turret, -112078));

    

    //Intake
    //controller2.button(2).and(() -> mode == false).whileTrue(Commands.runEnd(() -> manipulator.setPourcentage(0.4), () -> manipulator.setPourcentage(0), manipulator));
    //controller2.button(2).and(() -> mode == true).whileTrue(Commands.runEnd(() -> manipulator.setPourcentage(-0.4), () -> manipulator.setPourcentage(0), manipulator));
    controller2.button(2).and(() -> mode == false).onTrue(new PickupCommand(manipulator, true).andThen(Commands.runOnce(() -> hasGameObject = true)));
    controller2.button(2).and(() -> mode == true).onTrue(new PickupCommand(manipulator, false).andThen(Commands.runOnce(() -> hasGameObject = true)));

    //Outtake
    controller2.button(1).and(() -> mode == false).onTrue(Commands.sequence(Commands.runEnd(() -> manipulator.setPourcentage(0.4), () -> manipulator.setPourcentage(0), manipulator).withTimeout(0.5), Commands.runOnce(() -> hasGameObject = false)));
    controller2.button(1).and(() -> mode == true).onTrue(Commands.sequence(Commands.runEnd(() -> manipulator.setPourcentage(-0.4), () -> manipulator.setPourcentage(0), manipulator).withTimeout(0.5), Commands.runOnce(() -> hasGameObject = false)));
    

    //controller.b().onTrue(Commands.runOnce(() -> turret.resetEncoder(), turret));
    //controller.y().onTrue(Commands.runOnce(() -> turret.setMagicSetpoint(0), turret));
    //controller.b().onTrue(Commands.runOnce(() -> turret.setMagicSetpoint(229028), turret));
    /* Arm positions */
    //Low
    controller2.button(11).and(() -> mode == false).onTrue(Commands.sequence(new SetDrivetrainMaxOutputCommand(0.8, drivetrain), new SetRampEnabledCommand(drivetrain, true), new SetArmPositionCommand(arm, 200, 1950)));
    controller2.button(11).and(() -> mode == true).onTrue(Commands.sequence(new SetDrivetrainMaxOutputCommand(0.8, drivetrain), new SetRampEnabledCommand(drivetrain, true), new SetArmPositionCommand(arm, 200, 1930)));
    //Mid
    controller2.button(9).and(() -> mode == false).onTrue(Commands.sequence(new SetDrivetrainMaxOutputCommand(0.7, drivetrain), new SetRampEnabledCommand(drivetrain, false), new SetArmPositionCommand(arm, 1246, 1900)));
    controller2.button(9).and(() -> mode == true).onTrue(Commands.sequence(new SetDrivetrainMaxOutputCommand(0.7, drivetrain), new SetRampEnabledCommand(drivetrain, false), new SetArmPositionCommand(arm, 1246, 1900)));

    //High
    controller2.button(7).and(() -> mode == false).onTrue(Commands.sequence(new SetDrivetrainMaxOutputCommand(0.4, drivetrain), new SetRampEnabledCommand(drivetrain, false), new SetArmPositionCommand(arm, 2513, 1451)));
    controller2.button(7).and(() -> mode == true).onTrue(Commands.sequence(new SetDrivetrainMaxOutputCommand(0.4, drivetrain), new SetRampEnabledCommand(drivetrain, false), new SetArmPositionCommand(arm, 2325, 1500)));

    //Manual turret override
    controller.x().toggleOnTrue(Commands.run(() -> turret.setTurret(controller.getRightX()), turret));

    //Cancel all commands in case of uncontrollable behavior
    controller.leftBumper().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));

    //HP Shelf
    //controller2.button(8).and(() -> mode == false).onTrue(Commands.sequence(new SetDrivetrainMaxOutputCommand(0.4, drivetrain), new SetRampEnabledCommand(drivetrain, false), new SetArmPositionCommand(arm, 3016, 1099)));
    //controller2.button(8).and(() -> mode == true).onTrue(Commands.sequence(new SetDrivetrainMaxOutputCommand(0.4, drivetrain), new SetRampEnabledCommand(drivetrain, false), new SetArmPositionCommand(arm, 3016, 1099)));

    controller2.button(8).and(() -> mode == false).onTrue(new HPShelfCubeCommandGroup(drivetrain, arm, turret, manipulator).andThen(Commands.runOnce(() -> hasGameObject = true)));
    controller2.button(8).and(() -> mode == true).onTrue(new HPShelfConeCommandGroup(drivetrain, arm, turret, manipulator).andThen(Commands.runOnce(() -> hasGameObject = true)));

    //Stowed
    controller2.button(12).onTrue(Commands.sequence(new SetArmPositionCommand(arm, 701, 2926), new SetRampEnabledCommand(drivetrain, false), new SetDrivetrainMaxOutputCommand(0.5, drivetrain)));
    
    //Cone pickup (does not work)
    //controller2.button(10).onTrue(new SetArmPositionCommand(arm, 344, 1587));


    //controller2.button(4).onTrue(new PickupCommand(manipulator, false));
    controller2.button(4).toggleOnTrue(new TrackApriltagCommand(turret, 0));
    
    controller.a().whileTrue(Commands.parallel(new SetDrivetrainMaxOutputRememberCommand(drivetrain, 1), new SetRampEnabledRememberCommand(drivetrain, true)));


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

  public CommandXboxController getJoystick(){
    return controller;
  }

  public DrivetrainSubsystem getDrivetrain() {
    return drivetrain;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public LEDSubsystem getLEDs() {
    return leds;
  }
}
