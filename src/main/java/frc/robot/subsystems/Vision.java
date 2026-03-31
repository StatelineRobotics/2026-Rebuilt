// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.StatusSignal.SignalMeasurement;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

@Logged
public class Vision extends SubsystemBase {
  // Sim stuff
  VisionSystemSim visionSim = new VisionSystemSim("main");
  Supplier<Pose2d> poseSupplier;
  Supplier<SignalMeasurement<Angle>> pigeonRotationSupplier;
  private boolean useSim = false;

  private EstimateConsumer estimateConsumer;

  public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private Camera leftCamera = new Camera(
      "LowerLeft",
      new Transform3d(
          -0.229337, 0.331574, 0.490599, new Rotation3d(0.0, Math.toRadians(-15), Math.toRadians(90.0))),
      visionSim,
      useSim);

  private Camera backCamera = new Camera(
      "BackCam",
      new Transform3d(
          -0.29142425, 0.291663, 0.447, new Rotation3d(0.0, Math.toRadians(-15), Math.toRadians(180.0))),
      visionSim,
      useSim);

  private Camera frontCamera = new Camera(
      "FrontCam",
      new Transform3d(
          -0.29502575, 0.088337, 0.468, new Rotation3d(0.0, Math.toRadians(-15.0), Math.toRadians(0.0))),
      visionSim,
      useSim);

  private Camera rightCamera = new Camera(
      "RightCam",
      new Transform3d(-0.378, -0.238, 0.267, new Rotation3d(0.0, Math.toRadians(-15), Math.toRadians(-90.0))),
      visionSim,
      useSim);

  private Camera[] cameras = {leftCamera, backCamera, frontCamera, rightCamera};

  /** Creates a new Vision. */
  public Vision(
      EstimateConsumer poseConsumer,
      Supplier<Pose2d> simPoseSupplier,
      Supplier<SignalMeasurement<Angle>> headingSupplier) {
    estimateConsumer = poseConsumer;
    poseSupplier = simPoseSupplier;
    pigeonRotationSupplier = headingSupplier;

    if (Robot.isSimulation() && useSim) {
      visionSim.addAprilTags(kTagLayout);
    } else {
      visionSim = null;
    }

    SmartDashboard.putBoolean("UseCameras", true);
    SmartDashboard.putBoolean("Use PNP", false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation() && !useSim) {
      return;
    }
    if (!SmartDashboard.getBoolean("UseCameras", false)) {
      return;
    }
    for (var camera : cameras) {
      camera.update(estimateConsumer, pigeonRotationSupplier.get());
    }
  }

  @Override
  public void simulationPeriodic() {
    if (useSim) {
      visionSim.update(poseSupplier.get());
    }
  }

  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }

  @Logged
  public class Camera {
    private Transform3d transform;
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private EstimatedRobotPose estimatedPose;
    private boolean usedPose = false;
    private double xyStd = 0.0;
    private double angStd = 0.0;
    private boolean setIntrinsics = false;
    private Optional<Matrix<N3, N3>> cameraMatrix;
    private Optional<Matrix<N8, N1>> distCoeff;

    private static final double enabledXyStd = 0.075;
    private static final double enabledAngStd = 0.1;
    private static final double disabledXyStd = 0.4;
    private static final double disabledAngStd = 0.14;
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public Camera(
        String carmeraName, Transform3d robotToCameraTransform, VisionSystemSim visionSim, boolean useSim) {
      camera = new PhotonCamera(carmeraName);
      transform = robotToCameraTransform;
      poseEstimator = new PhotonPoseEstimator(kTagLayout, transform);

      if (Robot.isSimulation() && useSim) {
        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(10);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        PhotonCameraSim simCamera = new PhotonCameraSim(camera, cameraProp);
        visionSim.addCamera(simCamera, robotToCameraTransform);
      }

      cameraMatrix = camera.getCameraMatrix();
      distCoeff = camera.getDistCoeffs();
    }

    public void update(EstimateConsumer visionConsumer, SignalMeasurement<Angle> rotationSupplier) {
      if (!setIntrinsics) {
        cameraMatrix = camera.getCameraMatrix();
        distCoeff = camera.getDistCoeffs();
        setIntrinsics = cameraMatrix.isPresent() && distCoeff.isPresent();
      }
      usedPose = false;
      poseEstimator.addHeadingData(rotationSupplier.timestamp, new Rotation2d(rotationSupplier.value));

      // actual vision stuff
      for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
        var estimate = poseEstimator.estimateCoprocMultiTagPose(result);
        if (estimate.isEmpty()) {
          estimate = poseEstimator.estimateLowestAmbiguityPose(result);
          if (estimate.isEmpty()) {
            continue;
          }
        }

        if (setIntrinsics && SmartDashboard.getBoolean("Use PNP", false)) {
          estimate = poseEstimator.estimateConstrainedSolvepnpPose(
              result, cameraMatrix.get(), distCoeff.get(), estimate.get().estimatedPose, true, 0.5);
        }

        if (estimate.isEmpty()) {
          continue;
        }

        var tempEstimatedPose = estimate.get().estimatedPose;
        // Check if estimated pose is within the field
        if (tempEstimatedPose.getX() < (0 + Inches.of(33.0 / 2.0).in(Meters))
            || tempEstimatedPose.getX()
                > (kTagLayout.getFieldLength()
                    - Inches.of(33.0 / 2.0).in(Meters))
            || tempEstimatedPose.getY() < 0 && tempEstimatedPose.getY() > kTagLayout.getFieldWidth()
            || Math.abs(tempEstimatedPose.getZ()) > 0.1) {
          continue;
        }

        estimatedPose = estimate.get();
        var target = estimatedPose.targetsUsed.get(0);

        // Determine the distance from the camera to the tag.
        double distance = target.bestCameraToTarget.getTranslation().getNorm();

        // Calculate the pose estimation weights for X/Y location. As
        // distance increases, the tag is trusted exponentially less.
        xyStd = DriverStation.isEnabled() ? enabledXyStd : disabledXyStd;
        angStd = DriverStation.isEnabled() ? enabledAngStd : disabledAngStd;

        xyStd = xyStd * distance * distance;
        angStd = angStd * distance * distance;

        if (!RobotState.isDisabled()) {
          angStd = 0.3;
        }

        if (true) {
          usedPose = true;
          visionConsumer.accept(
              estimatedPose.estimatedPose.toPose2d(),
              estimatedPose.timestampSeconds,
              VecBuilder.fill(xyStd, xyStd, angStd));
        }
      }
    }
  }
}
