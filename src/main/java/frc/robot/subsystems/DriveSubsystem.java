package frc.robot.subsystems;
import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class DriveSubsystem  extends SubsystemBase {
    
  private final TalonSRX m_LeftMotor = new TalonSRX(DriveConstants.kLeftMotorPort);
  private final TalonSRX m_RightMotor = new TalonSRX(DriveConstants.kRightMotorPort);
  private final TalonSRX m_RearMotor = new TalonSRX(DriveConstants.kRearMotorPort);
  private final PigeonIMU IMU = new PigeonIMU(m_LeftMotor);

  public DriveSubsystem() {}


  public Command IMUgetHeadingMethodCommand() {
    return runOnce(() -> {
      IMU.getFusedHeading();
    });
  }

  public Command IMUzeroHeading() {
    return runOnce(() -> {
      IMU.setFusedHeading(0.0);
    });
  }

public void Drive(double x,double y,double r){

    x = exponential_drive(x, DriveConstants.kDriveExponetialConstant);
    y = exponential_drive(y, DriveConstants.kDriveExponetialConstant);
    r = exponential_drive(r, DriveConstants.kRotExponetialConstant);
  
    x = apply_deadband(x, DriveConstants.kDriveDeadbandXYConstant, DriveConstants.kDriveMaxXYConstant);
    y = apply_deadband(y, DriveConstants.kDriveDeadbandXYConstant, DriveConstants.kDriveMaxXYConstant);
    r = apply_deadband(r, DriveConstants.kDriveDeadbandRConstant, DriveConstants.kDriveMaxRConstant);
    
  
    double heading = IMU.getFusedHeading();
  
    //if(fieldOrient){    //calculate and apply field orientation
      double theta = heading * Math.PI/180;  //convert heading degrees to radians
      double yOut = x * Math.sin(theta) + y * Math.cos(theta);
      double xOut = x * Math.cos(theta) - y * Math.sin(theta);
      y = yOut;
      x = xOut;
    //}
  
    double right_Wheel = -0.5 * x - Math.sqrt(3)/2 * y + r;  //calculate 3 wheel holonomic
    double left_Wheel = -0.5 * x + Math.sqrt(3)/2 * y + r;
    double rear_Wheel = x + r;
  
    m_LeftMotor.set(ControlMode.PercentOutput, left_Wheel); 
    m_RightMotor.set(ControlMode.PercentOutput, right_Wheel); 
    m_RearMotor.set(ControlMode.PercentOutput, rear_Wheel); 
  }
  
  double exponential_drive(double input, double exponent) {
    return Math.copySign(Math.pow(Math.abs(input), exponent), input);
  }

  double apply_deadband(double value, double deadband, double maxMagnitude){
    double magnitude = Math.abs(value);
    if (magnitude > deadband){
      //if (maxMagnitude / deadband > 1.0E12):
          //if(value > 0.0):
              //return value - deadband
          //else:
              //return value + deadband    
      if (value > 0.0)
          return value * (maxMagnitude - deadband) + deadband;
      else
          return value * (maxMagnitude - deadband) - deadband;
    }
    else{
      return (0.0);
    }
  }

}