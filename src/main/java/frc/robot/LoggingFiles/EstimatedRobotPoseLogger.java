// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LoggingFiles;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.EstimatedRobotPose;

/** Add your docs here. */
@CustomLoggerFor(EstimatedRobotPose.class)
public class EstimatedRobotPoseLogger extends ClassSpecificLogger<EstimatedRobotPose> {

  public EstimatedRobotPoseLogger() {
    super(EstimatedRobotPose.class);
  }

  @Override
  protected void update(EpilogueBackend backend, EstimatedRobotPose estimatedPose) {
    if (estimatedPose == null) {
      return;
    }
    backend.log("Pose", estimatedPose.estimatedPose, Pose3d.struct);
    backend.log("Timestamp Seconds", estimatedPose.timestampSeconds);
    backend.log("Strategy", estimatedPose.strategy.toString());
    backend.log("Number Of Targets Used", estimatedPose.targetsUsed.size());
    // if (Epilogue.shouldLog(Importance.DEBUG)) {
    //   for (int i = 0; i < estimatedPose.targetsUsed.size(); i++) {
    //     backend.log(Integer.toString(i), estimatedPose.targetsUsed.get(i), PhotonTrackedTarget.proto);
    //   }
    // }
  }
}
