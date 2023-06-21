package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

// This class controls what happens in the auto.
public final class Autos {

  // Basic auto for the Drive Subsystem.
  public static CommandBase exampleAutoDrive(DriveSubsystem subsystem) {
    return Commands.sequence(subsystem.driveDistanceCommand(3, .5), 
                             subsystem.driveDistanceCommand(-3, 0.5));
  }

  // Basic  auto for the Arm Subsystem.
  // public static CommandBase exampleAutoArm(ArmSubsystem subsystem) {
  //   return Commands.sequence(subsystem.armToPositionPolar(36,52,0), 
  //                            subsystem.armToPositionPolar(36,36,0));
  // }

  // Handles an error in the auto for an Unsupported Operation Exception.
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
