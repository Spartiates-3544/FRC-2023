package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PickupCommand;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.commands.SetDrivetrainMaxOutputCommand;
import frc.robot.commands.SetRampEnabledCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class HPShelfCubeCommandGroup extends SequentialCommandGroup {
    public HPShelfCubeCommandGroup(DrivetrainSubsystem drivetrain, ArmSubsystem arm, TurretSubsystem turret, ManipulatorSubsystem manipulator) {
        addCommands(
            new SetDrivetrainMaxOutputCommand(0.7, drivetrain),
            new SetRampEnabledCommand(drivetrain, false),
            //Go to HP Shelf height
            new SetArmPositionCommand(arm, 2950, 1093),
            new PickupCommand(manipulator, true),
            //Wait 1 second before stowing
            new WaitCommand(1),
            new SetArmPositionCommand(arm, 701, 2926),
            new SetDrivetrainMaxOutputCommand(1, drivetrain),
            new SetRampEnabledCommand(drivetrain, true)
        );
    }
}