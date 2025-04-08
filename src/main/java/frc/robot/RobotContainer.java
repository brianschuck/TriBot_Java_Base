// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ledSubsystem;
import frc.robot.subsystems.PhotonSubsystem;

public class RobotContainer {

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public final ledSubsystem m_led = new ledSubsystem();
  public final PhotonSubsystem m_photon = new PhotonSubsystem();
    
  public final CommandXboxController m_driverController =
    new CommandXboxController(OperatorConstants.kDriverControllerPort);


  public RobotContainer() {

    m_driveSubsystem.setDefaultCommand(
      new RunCommand(() ->
        m_driveSubsystem.Drive(
          m_driverController.getLeftY(), 
          -m_driverController.getLeftX(), 
          m_driverController.getRightX(),
          true
        ),  
      m_driveSubsystem
    ));

    // m_led.setDefaultCommand(
    //   m_led.offLED()
    // );

    m_led.setDefaultCommand(
      new RunCommand(() ->
        m_led.CameraTargetLED(m_photon.cameraTargetValues(1)),
        m_led
      ));
    

    configureBindings();
  }

  private void configureBindings() {
    m_driverController.leftBumper().onTrue(m_driveSubsystem.IMUzeroHeading());
    m_driverController.a().whileTrue(m_led.blinkLED(Color.kGreen));
    m_driverController.x().whileTrue(m_led.TwoLEDColor(Color.kRed, Color.kBlue));


    m_driverController.b().whileTrue( 
      new RunCommand(() -> m_driveSubsystem.buttonRotateToTarget(
        m_driverController.getLeftY(), 
        -m_driverController.getLeftX(), 
        m_photon.getTargetYaw(),
        false
      )
      ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
