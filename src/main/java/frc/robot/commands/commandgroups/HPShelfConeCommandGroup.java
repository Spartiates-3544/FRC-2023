package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PickupCommand;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.commands.SetDrivetrainMaxOutputCommand;
import frc.robot.commands.SetRampEnabledCommand;
import frc.robot.commands.SetTurretPositionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class HPShelfConeCommandGroup extends SequentialCommandGroup {
    public HPShelfConeCommandGroup(DrivetrainSubsystem drivetrain, ArmSubsystem arm, TurretSubsystem turret, ManipulatorSubsystem manipulator) {
        addCommands(
            new SetDrivetrainMaxOutputCommand(0.7, drivetrain),
            new SetRampEnabledCommand(drivetrain, false),
            //Go to HP Shelf height
            new SetArmPositionCommand(arm, 3016, 1099),
            new PickupCommand(manipulator, false),
            //Wait 1 second before stowing
            new WaitCommand(1),
            new SetArmPositionCommand(arm, 200, 2868),
            new SetDrivetrainMaxOutputCommand(1, drivetrain),
            new SetRampEnabledCommand(drivetrain, true),
            // TODO Turn turret 180 degrees
            new SetTurretPositionCommand(turret, 0)
        );
    }
}