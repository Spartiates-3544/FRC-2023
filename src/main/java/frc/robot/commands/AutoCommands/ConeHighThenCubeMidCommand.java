package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PickupCommand;
import frc.robot.commands.RunPathCommand;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.commands.SetTurretPositionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ConeHighThenCubeMidCommand extends SequentialCommandGroup {
    String[] path = {"/home/lvuser/deploy/paths/2piece1.json"};
    String[] path2 = {"/home/lvuser/deploy/paths/2piece3.json"};

    public ConeHighThenCubeMidCommand(DrivetrainSubsystem drivetrain, ArmSubsystem arm, TurretSubsystem turret, ManipulatorSubsystem manipulator) {
        addCommands(
            new SetArmPositionCommand(arm, 200, 2868).withTimeout(1),
            new SetTurretPositionCommand(turret, 229028),
            new SetArmPositionCommand(arm, 2800, 1087),
            Commands.runEnd(() -> manipulator.setPourcentage(-0.4), () -> manipulator.setPourcentage(0), turret).withTimeout(0.5),
            Commands.parallel(new SetArmPositionCommand(arm, 200, 2868), new SetTurretPositionCommand(turret, 0), new RunPathCommand(drivetrain, path).withTimeout(5)),
            Commands.parallel(new SetArmPositionCommand(arm, 200, 1910), new PickupCommand(manipulator, true).withTimeout(2)),
            new SetArmPositionCommand(arm, 200, 2868).withTimeout(2),
            Commands.parallel(new SetTurretPositionCommand(turret, 229028), new RunPathCommand(drivetrain, path2).withTimeout(4)),
            new SetArmPositionCommand(arm, 1246, 1900),
            Commands.runEnd(() -> manipulator.setPourcentage(0.4), () -> manipulator.setPourcentage(0), turret).withTimeout(0.5)
        );
    }
}
