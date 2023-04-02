package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RunPathCommand;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.commands.SetStage1Command;
import frc.robot.commands.SetStage2Command;
import frc.robot.commands.SetTurretPositionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ConeHautSortirCommand extends SequentialCommandGroup {
    String[] path = {"/home/lvuser/deploy/paths/AutonomeReculer.json"};
    public ConeHautSortirCommand(DrivetrainSubsystem drivetrain, ArmSubsystem arm, TurretSubsystem turret, ManipulatorSubsystem manipulator) {
        addCommands(
            new SetArmPositionCommand(arm, 200, 2868).withTimeout(1),
            new SetTurretPositionCommand(turret, 229028),
            Commands.parallel(new SetStage1Command(arm, 2800), Commands.sequence(Commands.waitSeconds(1), new SetStage2Command(arm, 1120))),
            /*
            new SetStage1Command(arm, 2800),
            Commands.waitSeconds(0.5),
            new SetStage2Command(arm, 1120),
            */
            Commands.runEnd(() -> manipulator.setPourcentage(-0.4), () -> manipulator.setPourcentage(0), turret).withTimeout(0.5),
            Commands.parallel(new RunPathCommand(drivetrain, path).withTimeout(5), Commands.sequence(Commands.waitSeconds(0.5), new SetArmPositionCommand(arm, 200, 2868)))
            //Commands.parallel(new SetArmPositionCommand(arm, 200, 2868)/*,new RunPathCommand(drivetrain, path).withTimeout(5) */)
        );
    }


}
