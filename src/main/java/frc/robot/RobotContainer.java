// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

  private final DriveSubsystem m_driveSubsystem = 
    new DriveSubsystem();
    
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


    configureBindings();
  }

  private void configureBindings() {
    m_driverController.leftBumper().onTrue(m_driveSubsystem.IMUzeroHeading());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
