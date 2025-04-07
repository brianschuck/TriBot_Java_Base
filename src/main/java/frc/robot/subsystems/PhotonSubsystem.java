// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonSubsystem extends SubsystemBase{
  private final PhotonCamera camera1;
  public PhotonSubsystem() {
  // Implementation of subsystem constructor goes here.
  camera1 = new PhotonCamera("Cam1");
}

public Command SetDriverModeFalseCommand() {
    //if(camera1.GetDriverMode() == true)
    return run(() -> camera1.setDriverMode(false));
}

public Command SetPipelineIndexCommand() {
    //if(camera1.GetPipelineIndex() != 1)
    return run(() -> camera1.setPipelineIndex(1)); 
}


  public double cameraTargetValues(){
    PhotonPipelineResult result = camera1.getLatestResult();
    double targetVal = 0;
       if (result.hasTargets()) {
          PhotonTrackedTarget target = result.getBestTarget(); 
          //if (target.GetFiducialId() == 1)  {   //only if we want a specific ID
            targetVal = target.getYaw();
          //}
        }
        else{
          targetVal = 100.0;  //return 100 when no target found
        }
    return targetVal;
  }




void Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
}