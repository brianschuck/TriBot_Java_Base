package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.ledConstants;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;


public class ledSubsystem extends SubsystemBase{
  
  private AddressableLEDBuffer m_ledBuffer;
  @SuppressWarnings("IOCheck")
  private AddressableLED m_led;
  
  public ledSubsystem() {
    setDefaultCommand(null);
    m_led = new AddressableLED(ledConstants.kPort);
    m_ledBuffer = new AddressableLEDBuffer(ledConstants.kLength);
    m_led.setLength(ledConstants.kLength);
    m_led.setData(m_ledBuffer);
    m_led.start();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_led.setData(m_ledBuffer);
  }

  public Command TwoLEDColor(Color color1, Color color2){
    return run(() -> LEDPattern
      .gradient(GradientType.kContinuous, color1, color2)
      .breathe(Seconds.of(4))
      .applyTo(m_ledBuffer));
  }

  public Command blinkLED(Color color){
    return run(() -> LEDPattern
      .solid(color)
      .blink(Seconds.of(1))
      .applyTo(m_ledBuffer)
    );
  }

  public Command offLED(){
    return run(() -> LEDPattern
      .solid(Color.kBlack)
      .applyTo(m_ledBuffer)
      );
  }

  public Command CameraTargetLED(double targetValue){
    int bufferSize = m_ledBuffer.getLength();
    if(targetValue != 101.0){
      if(targetValue < 100.0){  // 999 is returned if there is no valid target detected from the camera
          long targetValInt = Math.round(targetValue); //round the double to the nearest INT 
            targetValInt += 18;  // Only using 36 LEDs
        for(int i = 0; i < bufferSize; i++){
          if(targetValInt == i){
            m_ledBuffer.setRGB(i ,0, 255, 0);  //set the led green if its array value matches the target
          }
          else
          m_ledBuffer.setRGB(i, 0, 0, 0);   //set the rest to off
        }
      }
      else{     //target not valid
        for(int i = 0; i < bufferSize; i++){
          m_ledBuffer.setRGB(i, 25, 0, 0);   //set all to red
        }
      }
    }
    return run(() -> m_led.setData(m_ledBuffer)
    );
  }


}