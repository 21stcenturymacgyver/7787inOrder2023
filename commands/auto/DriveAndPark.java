package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MotorCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MotorAutoTest;

public class DriveAndPark extends SequentialCommandGroup{
    public DriveAndPark(MotorAutoTest m_DriveSubsystem) {
        
        addCommands(
            new MotorCmd(m_DriveSubsystem,1),
            new MotorCmd(m_DriveSubsystem, 0)
            );
            
            
            
    }

    
    
}
