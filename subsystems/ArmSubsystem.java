package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ArmSubsystem extends SubsystemBase {

  // Define all of the motors
  SparkmaxMotor AzimuthMotor = new SparkmaxMotor(ArmConstants.AZIMUTH_MOTOR_ID);
  SparkmaxMotor ShoulderMotor = new SparkmaxMotor(ArmConstants.SHOULDER_MOTOR_ID);
  SparkmaxMotor ElbowMotor = new SparkmaxMotor(ArmConstants.ELBOW_MOTOR_ID);
  SparkmaxMotor ClawMotor = new SparkmaxMotor(ArmConstants.CLAW_MOTOR_ID); 

  public float[] getArmPosition(float Theta1, float Theta2) {

    // Calculations done in Radians
    float Hypotenus = 2 * ArmConstants.ARM_LENGTH1 * (float) Math.sin(Theta2 / 2 * Math.PI / 180);
    float Theta1_1 = (float) Math.asin((ArmConstants.ARM_LENGTH1 / Hypotenus) * Math.sin(Theta2 * Math.PI / 180));
    float Theta1_2 = Theta1 - Theta1_1;
  
    // Define an arry of values
    float[] HeightAndRadius = new float[1];
    HeightAndRadius[0] = Hypotenus * (float) Math.cos(Theta1_2 * Math.PI / 180); 
    HeightAndRadius[1] = Hypotenus * (float) Math.sin(Theta1_2 * Math.PI / 180);
    
    // Return values
    return HeightAndRadius;
  }
    
  public float[] GetArmAngle( float Hypotenus, float Theta1, float Theta2) {
    float[] Thetas = new float[2]; // make it [ conversion factor * this.ShoulderMotor.getEncoderValue(), other one];

    // Checks to see if the angles are the correct ones
    if (Theta1 > ArmConstants.THETA1_MIN || Theta1 < ArmConstants.THETA1_MAX || Theta2 > ArmConstants.THETA2_MIN || Theta2 < ArmConstants.THETA2_MAX) { 
      // Calculations done in Radians
      
      Thetas[0] = (float) Math.acos((Math.pow(ArmConstants.ARM_LENGTH1, 2) + Math.pow(Hypotenus, 2) - Math.pow(ArmConstants.ARM_LENGTH2, 2)) / (2 * ArmConstants.ARM_LENGTH1 * Hypotenus));
      Thetas[1] = (float) Math.acos((Math.pow(ArmConstants.ARM_LENGTH1, 2) + Math.pow(ArmConstants.ARM_LENGTH2, 2) - Math.pow(Hypotenus, 2)) / (2 * ArmConstants.ARM_LENGTH1 * ArmConstants.ARM_LENGTH2)); 
      
      // Return the values.
      }
    // Error handling for invalid Theta1 angles.
    else if (Theta1 < ArmConstants.THETA1_MIN || Theta1 > ArmConstants.THETA1_MAX) {
      SmartDashboard.putString("1", "Please enter an angle between 30 and 135 degrees for Theta1.");
    }
    // Error handling for invalid Theta2 angles.
    else if (Theta2 < ArmConstants.THETA2_MIN || Theta2 > ArmConstants.THETA2_MAX) {
      SmartDashboard.putString("2", "Please enter an angle between 30 and 180 degrees for Theta2.");
    }
    // Error handling for incorrect data types such as str, or array.
    else {
      SmartDashboard.putString("3", "Please enter a valid angle of type [Int], not a string, array, boolean, list, string, or any other datatype.");
    }

    return Thetas;

  }

  public CommandBase armToPositionPolar(double Radius, double Height, double Azimuth) {
    return run(()->getArmPosition(0, 0));
  } 
}