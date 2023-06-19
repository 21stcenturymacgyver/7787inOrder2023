// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAutoDrive(DriveSubsystem subsystem) {
    return Commands.sequence(subsystem.driveDistanceCommand(3, .5), subsystem.driveDistanceCommand(-3, 0.5));
  }

  public static CommandBase exampleAutoArm(ArmSubsystem subsystem) {
    return Commands.sequence(subsystem.armToPositionPolar(36,52,0), subsystem.armToPositionPolar(36,36,0));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
