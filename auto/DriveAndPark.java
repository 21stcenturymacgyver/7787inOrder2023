package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveBackTime;
import frc.robot.subsystems.DriveSubsystem;

public class DriveAndPark extends SequentialCommandGroup{
    public DriveAndPark(DriveSubsystem m_DriveSubsystem) {
        addCommands(
            new DriveBackTime(m_DriveSubsystem, -0.3, 4),
            new DriveBackTime(m_DriveSubsystem, 0, 1)
        );
    }
}
