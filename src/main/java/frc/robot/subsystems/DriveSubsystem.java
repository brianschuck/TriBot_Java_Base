package frc.robot.subsystems;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
      IMU.getYaw();
    });
  }

  public Command IMUzeroHeading() {
    return runOnce(() -> {
      IMU.setYaw(0.0);
    });
  }

  public void buttonRotateToTarget(double x, double y, double r, boolean driverControl){
    x = 0;
    y = 0;
    if(r == 999){ // 999 is no valid target from the camera
      r = 0;
    }
    else{ //need a pid controller here
      if(r > 0){
        r = 0.085;
      }
      else{
        r= -0.085;
      }
    }
      Drive(x, y, r, driverControl);
  }

  public void Drive(double x,double y,double r, boolean driverControl){
    if(driverControl){
      x = exponential_drive(x, DriveConstants.kDriveExponetialConstant);
      y = exponential_drive(y, DriveConstants.kDriveExponetialConstant);
      r = exponential_drive(r, DriveConstants.kRotExponetialConstant);
  
      x = apply_deadband(x, DriveConstants.kDriveDeadbandXYConstant, DriveConstants.kDriveMaxXYConstant);
      y = apply_deadband(y, DriveConstants.kDriveDeadbandXYConstant, DriveConstants.kDriveMaxXYConstant);
      r = apply_deadband(r, DriveConstants.kDriveDeadbandRConstant, DriveConstants.kDriveMaxRConstant);
      
      double heading = IMU.getYaw();
    
      double theta = heading * Math.PI/180;  //convert heading degrees to radians
      double yOut = x * Math.sin(theta) + y * Math.cos(theta);
      double xOut = x * Math.cos(theta) - y * Math.sin(theta);
      y = yOut;
      x = xOut;
    }

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
      //prevents some obscure error that can be cause by an large value
      //if (maxMagnitude / deadband > 1.0E12):   
          //if(value > 0.0):
              //return value - deadband
          //else:
              //return value + deadband    
      if (value > 0.0)  //positive value. calculate accordingly
          return value * (maxMagnitude - deadband) + deadband;
      else    //negative value
          return value * (maxMagnitude - deadband) - deadband;
    }
    else{
      return (0.0);   // inside of the deadband. Do nothing
    }
  }

}