// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVision;

public class SUB_PhotonVision extends SubsystemBase {
  private static SUB_PhotonVision INSTANCE = null;
  
  private final PhotonCamera cam1 = new PhotonCamera(PhotonVision.kCamName);
  private final PhotonCamera cam2 = new PhotonCamera(PhotonVision.kCam2Name);
  private PhotonTrackedTarget cam1BestTarget;
  private PhotonTrackedTarget cam2BestTarget;
  private final PhotonPoseEstimator poseEstimator1;
  private final PhotonPoseEstimator poseEstimator2;
  public AprilTagFieldLayout at_field;

  private VisionSystemSim visionSim;
  private PhotonCameraSim cam1Sim;
  private PhotonCameraSim cam2Sim;

  private final StructPublisher<Pose2d> cam1Publisher = NetworkTableInstance.getDefault()
      .getStructTopic("Vision/Cam1Pose", Pose2d.struct).publish();
  private final StructPublisher<Pose2d> cam2Publisher = NetworkTableInstance.getDefault()
      .getStructTopic("Vision/Cam2Pose", Pose2d.struct).publish();

  public static SUB_PhotonVision getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_PhotonVision();
    }
    return INSTANCE;
  }

  private SUB_PhotonVision() {
    at_field =  AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark); // TODO: Change for diff events

    cam1.setPipelineIndex(0);
    cam2.setPipelineIndex(0);

    PoseStrategy strategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    if (RobotBase.isSimulation()) {
        strategy = PoseStrategy.MULTI_TAG_PNP_ON_RIO;
    }

    poseEstimator1 = new PhotonPoseEstimator(at_field, strategy,
        PhotonVision.kRobotToCamera1);
    poseEstimator2 = new PhotonPoseEstimator(at_field, strategy,
         PhotonVision.kRobotToCamera2);
    poseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    poseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(at_field);

      SimCameraProperties cameraProp = new SimCameraProperties();
      // Arducam OV9281: 1280 x 800 resolution with a 70 degree diagonal FOV.
      cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
      cameraProp.setCalibError(0.25, 0.08);
      cameraProp.setFPS(20);
      cameraProp.setAvgLatencyMs(35);
      cameraProp.setLatencyStdDevMs(5);

      cam1Sim = new PhotonCameraSim(cam1, cameraProp);
      cam2Sim = new PhotonCameraSim(cam2, cameraProp);

      visionSim.addCamera(cam1Sim, PhotonVision.kRobotToCamera1);
      visionSim.addCamera(cam2Sim, PhotonVision.kRobotToCamera2);
      
      cam1Sim.enableDrawWireframe(true);
      cam2Sim.enableDrawWireframe(true);
    }
  }

  public void updateSimPose(Pose2d robotPose) {
    if (RobotBase.isSimulation()) {
        visionSim.update(robotPose);
    }
  }

  private boolean isPoseValid(EstimatedRobotPose pose, String camName) {
      // 1. Height Check (Robot shouldn't fly)
      if (Math.abs(pose.estimatedPose.getZ()) > PhotonVision.kMaxZError) {
          String msg = "REJECTED: Too High (" + pose.estimatedPose.getZ() + "m)";
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Vision/" + camName + "/Status", msg);
          System.out.println("[" + camName + "] " + msg);
          return false;
      }

      // 2. Field Bounds Check (Roughly)
      if (pose.estimatedPose.getX() < -1.0 || pose.estimatedPose.getX() > frc.robot.Constants.Field.fieldLength + 1.0) {
          String msg = "REJECTED: Out of Bounds (X)";
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Vision/" + camName + "/Status", msg);
          System.out.println("[" + camName + "] " + msg);
          return false;
      }
      if (pose.estimatedPose.getY() < -1.0 || pose.estimatedPose.getY() > frc.robot.Constants.Field.fieldWidth + 1.0) {
          String msg = "REJECTED: Out of Bounds (Y)";
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Vision/" + camName + "/Status", msg);
          System.out.println("[" + camName + "] " + msg);
          return false;
      }

      // 3. Distance Check (Average distance to tags)
      double totalDist = 0;
      int tagCount = 0;
      for (PhotonTrackedTarget target : pose.targetsUsed) {
          totalDist += target.getBestCameraToTarget().getTranslation().getNorm();
          tagCount++;
      }
      if (tagCount > 0 && (totalDist / tagCount) > PhotonVision.kMaxDistance) {
          String msg = "REJECTED: Too Far (" + (totalDist/tagCount) + "m)";
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Vision/" + camName + "/Status", msg);
          System.out.println("[" + camName + "] " + msg);
          return false;
      }

      edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Vision/" + camName + "/Status", "ACCEPTED");
      return true;
  }

  public Optional<EstimatedRobotPose> getCam1Pose() {
    List<PhotonPipelineResult> results1 = cam1.getAllUnreadResults();
  
    Optional<EstimatedRobotPose> finalPose1 = Optional.empty();
    for (PhotonPipelineResult result : results1) {
      if (result.hasTargets()) {
        cam1BestTarget = result.getBestTarget();
        // Filter: Ambiguity Check
        if (cam1BestTarget.getPoseAmbiguity() > PhotonVision.kMaxAmbiguity) continue;
        
        Optional<EstimatedRobotPose> pose = poseEstimator1.update(result);
        if (pose.isPresent() && isPoseValid(pose.get(), "Cam1")) {
            finalPose1 = pose;
            cam1Publisher.set(pose.get().estimatedPose.toPose2d());
        }
      }
    }
    return finalPose1;
  }

  public Optional<EstimatedRobotPose> getCam2Pose() {
    List<PhotonPipelineResult> results2 = cam2.getAllUnreadResults();
    Optional<EstimatedRobotPose> finalPose2 = Optional.empty();
    for (PhotonPipelineResult result : results2) {
      if (result.hasTargets()) {
        cam2BestTarget = result.getBestTarget();
         // Filter: Ambiguity Check
         if (cam2BestTarget.getPoseAmbiguity() > PhotonVision.kMaxAmbiguity) {
             edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Vision/Cam2/Status", "REJECTED: High Ambiguity");
             continue;
         }

        Optional<EstimatedRobotPose> pose = poseEstimator2.update(result);
        if (pose.isPresent() && isPoseValid(pose.get(), "Cam2")) {
            finalPose2 = pose;
            cam2Publisher.set(pose.get().estimatedPose.toPose2d());
        }
      }
    }
    return finalPose2;
  }

  public PhotonTrackedTarget getCam1BestTarget() {
    return cam1BestTarget;
  }

  public PhotonTrackedTarget getCam2BestTarget() {
    return cam2BestTarget;
  }

  public double getTargetYaw(PhotonTrackedTarget target) {
    return target.getYaw();
  }

  public double getTargetPitch(PhotonTrackedTarget target) {
    return target.getPitch();
  }

  public double getTargetArea(PhotonTrackedTarget target) {
    return target.getArea();
  }

  public int getId(PhotonTrackedTarget target) {
    return target.getFiducialId();
  }

  @Override
  public void periodic() {

  }
}
