package frc.robot.subsystems;

//import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
//import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController;
import java.util.function.DoubleSupplier;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ArmSubsystem extends SubsystemBase {

  // Define all of the motors
  SparkmaxMotor AzimuthMotor = new SparkmaxMotor(ArmConstants.AZIMUTH_MOTOR_ID, ArmConstants.AZIMUTH_LIMIT_SWITH_DIRECTION);
  SparkmaxMotor ShoulderMotor = new SparkmaxMotor(ArmConstants.SHOULDER_MOTOR_ID, ArmConstants.SHOULDER_LIMIT_SWITCH_DIRECTION);
  SparkmaxMotor ElbowMotor = new SparkmaxMotor(ArmConstants.ELBOW_MOTOR_ID, ArmConstants.ELBOW_LIMIT_SWITCH_DIRECTION);
  SparkmaxMotor ClawMotor = new SparkmaxMotor(ArmConstants.CLAW_MOTOR_ID, ArmConstants.CLAW_LIMIT_SWITCH_DIRECTION); 

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

    // Checks to see if the angles are the correct ones.
    if (Theta1 > ArmConstants.THETA1_MIN || Theta1 < ArmConstants.THETA1_MAX || Theta2 > ArmConstants.THETA2_MIN || Theta2 < ArmConstants.THETA2_MAX) {

      // Calculations done in Radians
      Thetas[0] = (float) Math.acos((Math.pow(ArmConstants.ARM_LENGTH1, 2) + Math.pow(Hypotenus, 2) - Math.pow(ArmConstants.ARM_LENGTH2, 2)) / (2 * ArmConstants.ARM_LENGTH1 * Hypotenus));
      Thetas[1] = (float) Math.acos((Math.pow(ArmConstants.ARM_LENGTH1, 2) + Math.pow(ArmConstants.ARM_LENGTH2, 2) - Math.pow(Hypotenus, 2)) / (2 * ArmConstants.ARM_LENGTH1 * ArmConstants.ARM_LENGTH2)); 
      
      }
    
    return Thetas;

  }

  private void RunAllMotors(double m_Aziumuth, double m_Shoulder, double m_Elbow, double m_Claw) {

    this.AzimuthMotor.runToposition(m_Aziumuth);
    this.ShoulderMotor.runToposition(m_Shoulder);
    this.ElbowMotor.runToposition(m_Elbow);
    this.ClawMotor.runToposition(m_Claw);

  }

  public CommandBase armToPositionPolar(double Radius, double Height, double Azimuth) {

    return run(()->getArmPosition(0, 0));

  } 

  public CommandBase joystickMotorCommand(DoubleSupplier m_Aziumuth,DoubleSupplier m_Shoulder,DoubleSupplier m_Elbow,DoubleSupplier m_Claw) {

    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> this.RunAllMotors(m_Aziumuth.getAsDouble(),m_Shoulder.getAsDouble(),m_Elbow.getAsDouble(),m_Claw.getAsDouble()))
        .withName("Joystick Arm Motor");

  }

  public CommandBase RunArmToPositionCommand( float c_theta1, float c_theta2) {

       return run(()->RunArmToPosition(c_theta1, c_theta2));

  }
  public void RunArmToPosition( float m_theta1, float m_theta2) {
    if (m_theta2<30){
      m_theta1= Math.max(98,m_theta1);//set  minimum angle to 98 when arm is within robot frame perimeter
    }
    m_theta2=Math.max(m_theta2, 9);// prevent the elbow from trying to close less then start
    if(m_theta1<97){
      m_theta2=Math.max(m_theta2, 30);//prevent bringing the arm into the perimeter from the side
    }

    ShoulderMotor.runToposition((-m_theta1+ArmConstants.THETA1_START)/ArmConstants.SHOULDER_DEGREES_PER_REVOLUTION);
    ElbowMotor.runToposition((m_theta2-ArmConstants.THETA2_START)/ArmConstants.ELBOW_DEGREES_PER_REVOLUTION);
    
  }


  public CommandBase RunArmToPositionCommand( float Theta1, float Theta2, float Azimuth)  {
    
    
    return run(()->RunArmToPosition(Theta1, Theta2,Azimuth));

  }
  public void RunArmToPosition( float Theta1, float Theta2, float Azimuth) {

    RunArmToPosition(Theta1, Theta2);
    AzimuthMotor.runToposition(Azimuth/ArmConstants.AZIMUTH_DEGREES_PER_REVOLUTION);//*90/153.6);
    
  }

}