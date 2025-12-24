// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.SUB_PhotonVision;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class CMD_PathfindAlgaeAlign extends Command {

  Command pathfindingCommand;
  SUB_PhotonVision photonVision;
  CommandSwerveDrivetrain drivetrain;

  HashMap<Integer, Translation2d> redLeft = new HashMap<>();
  HashMap<Integer, Translation2d> redRight = new HashMap<>();
  HashMap<Integer, Translation2d> blueLeft = new HashMap<>();
  HashMap<Integer, Translation2d> blueRight = new HashMap<>();

  /** Creates a new CMD_PathfindReefAlign. */
  public CMD_PathfindAlgaeAlign(CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision) {
    this.photonVision = photonVision;
    this.drivetrain = drivetrain;

    redRight.put(7, new Translation2d(14.341348, 4.0259));
redLeft.put(7, new Translation2d(14.341348, 4.0259));
redRight.put(8, new Translation2d(13.699871, 5.135929553296214));
redLeft.put(8, new Translation2d(13.699871, 5.135929553296214));
redRight.put(9, new Translation2d(12.417933, 5.135929553296214));
redLeft.put(9, new Translation2d(12.417933, 5.135929553296214));
redRight.put(10, new Translation2d(11.776455999999998, 4.0259));
redLeft.put(10, new Translation2d(11.776455999999998, 4.0259));
redRight.put(11, new Translation2d(12.417933, 2.9158704467037855));
redLeft.put(11, new Translation2d(12.417933, 2.9158704467037855));
redRight.put(6, new Translation2d(13.699871, 2.9158704467037855));
redLeft.put(6, new Translation2d(13.699871, 2.9158704467037855));
blueRight.put(21, new Translation2d(5.771896, 4.0259));
blueLeft.put(21, new Translation2d(5.771896, 4.0259));
blueRight.put(20, new Translation2d(5.130165, 5.135929553296214));
blueLeft.put(20, new Translation2d(5.130165, 5.135929553296214));
blueRight.put(19, new Translation2d(3.848480999999999, 5.135929553296214));
blueLeft.put(19, new Translation2d(3.848480999999999, 5.135929553296214));
blueRight.put(18, new Translation2d(3.20675, 4.0259));
blueLeft.put(18, new Translation2d(3.20675, 4.0259));
blueRight.put(17, new Translation2d(3.8484809999999987, 2.9158704467037855));
blueLeft.put(17, new Translation2d(3.8484809999999987, 2.9158704467037855));
blueRight.put(22, new Translation2d(5.130165, 2.9158704467037855));
blueLeft.put(22, new Translation2d(5.130165, 2.9158704467037855));
    
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d tagPose = new Pose2d();
    Integer targetId = 7;


    List<Integer> targetTagSet;
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    HashMap<Integer, Translation2d> selectedMap;
    if (alliance.isPresent()) {
      targetTagSet =
          alliance.get() == DriverStation.Alliance.Red ? Arrays.asList(7, 8, 9, 10, 11, 6)
              : Arrays.asList(21, 20, 19, 18, 17, 22);

        if (alliance.get() == DriverStation.Alliance.Red) {
          selectedMap = redLeft;
        } else {
          selectedMap = blueLeft;
        }
    } else {
      return;
    }


    double minDistance = Double.MAX_VALUE;
    for (int tag : targetTagSet) {
      Pose2d pose = photonVision.at_field.getTagPose(tag).orElse(new Pose3d()).toPose2d();
      Translation2d translate = pose.minus(drivetrain.getPose()).getTranslation();
      double distance = translate.getNorm();

      if (distance < minDistance) {
        tagPose = pose;
        targetId = tag;
        minDistance = distance;
      }
    }

    PathConstraints constraints = new PathConstraints(
    3.0, 2.1,
    Units.degreesToRadians(540), Units.degreesToRadians(720));


    Translation2d translate = selectedMap.get(targetId);
    Pose2d pose = new Pose2d(translate.getX(), translate.getY(), tagPose.getRotation().plus(Rotation2d.fromRadians(Math.PI)));
    drivetrain.publisher1.set(pose);
    pathfindingCommand = AutoBuilder.pathfindToPose(pose, constraints);

    pathfindingCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathfindingCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathfindingCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathfindingCommand.isFinished();
  }
}