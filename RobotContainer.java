// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import frc.robot.Constants.ButtonMappings;
import frc.robot.commands.FollowTargets;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.PS4Controller;
//import edu.wpi.first.wpilibj.Joystick;
// import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.Autos;
// import frc.robot.commands.ExampleCommand;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem DriveSubsystem = new DriveSubsystem();
  private final LimelightSubsystem LimelightSubsystem = new LimelightSubsystem();
  private final ArmSubsystem ArmSubsystem = new ArmSubsystem();

  // **** feb 24
  private final SparkmaxMotor testSparkmaxMotor = new SparkmaxMotor(4); // **** Feb 24
  
  private final PS4Controller ps4 = new PS4Controller(Constants.Driver_Controller);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
        
  }

  public LimelightSubsystem getLimelightSubsystem(){
    return this.LimelightSubsystem;
  }
  public ArmSubsystem getArmSubsystem(){
    return this.ArmSubsystem;
  }
  public DriveSubsystem getDriveSubsystem(){
    return this.DriveSubsystem;
  }
  public SparkmaxMotor getMotorTest(){ // **** feb 24
    return this.testSparkmaxMotor;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    // PS4 bindings
    new JoystickButton(this.ps4, ButtonMappings.R1).whileTrue((this.LimelightSubsystem.setPipeline9Command()));//changed to whileTrue
    new JoystickButton(this.ps4, ButtonMappings.R1).whileFalse(new StopMotor(this.DriveSubsystem));//changed to whileFalse


    //default commands
    this.LimelightSubsystem.setDefaultCommand(
      this.LimelightSubsystem.checkForTargetsCommand()
    );

    this.DriveSubsystem.setDefaultCommand(
          this.DriveSubsystem.arcadeDriveSquaredCommand(
              () -> -this.ps4.getLeftY(), () -> -this.ps4.getLeftX())
    );

    this.testSparkmaxMotor.setDefaultCommand(// **** Feb 24
          this.testSparkmaxMotor.joystickMotorCommand(
              () -> Math.abs(this.ps4.getRightY()*20))
    );


    //Trigger bindings (events)
    //new Trigger(LimeLight::hasTargets).onTrue(LimeLight.getTargetsCommand());
    new Trigger(this.LimelightSubsystem::hasTargets).whileTrue( new FollowTargets(this.DriveSubsystem,this.LimelightSubsystem));

    
  }
// 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
  //}
}
