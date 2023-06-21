// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.NavXSubsystem;;

public class Auto extends CommandBase {

  DriveSubsystem drive_auto;
  ArmSubsystem arm_auto;
  NavXSubsystem navX_auto;

  /** Creates a new Auto. */
  public Auto(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {

    drive_auto = driveSubsystem;
    arm_auto = armSubsystem;
    addRequirements(drive_auto, arm_auto);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //navX_auto.ahrsInit();
    //auto.initDrive();
    SmartDashboard.putString("b", "Auto Init");
  }
  //public void onceComplete() {
    //drive_auto.balanceRobot();


  //   drive_auto.driveToPositionCommand(0.0);
  //       SmartDashboard.putString("a", "Auto Complete");
  //       SmartDashboard.putString("b", "Pitch" + navX_auto.navXPitch());
  //       SmartDashboard.putString("c", "Yaw" + navX_auto.navXYaw());
  //       SmartDashboard.putString("d", "Roll" + navX_auto.navXRoll());
  // }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive_auto.shiftHigh();
    drive_auto.driveToPositionCommand(-180.0);
    // drive_auto.driveToPositionCommand(40.0);
    // drive_auto.shiftLow();
    // while(Math.abs(navX_auto.navXPitch()) < 10) {
    //     drive_auto.driveToPositionCommand(4.0);
    // }
    // while(Math.abs(navX_auto.navXPitch()) > 0) {
    //     drive_auto.driveToPositionCommand(1.0);
    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
