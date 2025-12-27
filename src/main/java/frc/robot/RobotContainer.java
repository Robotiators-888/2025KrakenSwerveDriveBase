// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import org.json.simple.parser.ParseException;
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Field;
import frc.robot.Constants.Operator;
import frc.robot.commands.CMD_OldPathfindReefAlign;
import frc.robot.commands.CMD_PathfindAlgaeAlign;
import frc.robot.commands.CMD_PathfindReefAlign;
import frc.robot.generated.TunerConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;


import frc.robot.subsystems.SUB_PhotonVision;


import frc.robot.utils.Elastic;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        private static final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
        private static final SUB_PhotonVision photonVision = SUB_PhotonVision.getInstance();

        private final SendableChooser<Command> autoChooser;
        public static PowerDistribution powerDistribution = new PowerDistribution();
        private static String autoName, newAutoName;
        Optional<Alliance> lastAlliance;
        Optional<Alliance> alliance;
        public static Field2d autoField = new Field2d();
        public int listIndex = 0;
        public int targetId = 7;

        // Replace with CommandPS4Controller or CommandJoystick if needed
        private final CommandXboxController Driver1 =
                        new CommandXboxController(Operator.kDriver1ControllerPort);

        private final CommandXboxController Driver2 =
                        new CommandXboxController(Operator.kDriver2ControllerPort);

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Operator.kDriveDeadband)
            .withRotationalDeadband(Operator.kDriveDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive.withVelocityX(-deadbandCompensate(Driver1.getLeftY()) * TunerConstants.kSpeedAt12VoltsMps)
                        .withVelocityY(-deadbandCompensate(Driver1.getLeftX()) * TunerConstants.kSpeedAt12VoltsMps)
                        .withRotationalRate(-deadbandCompensate(Driver1.getRightX()) * Math.PI * 2)));

                
                NamedCommands.registerCommand("ReachedTarget", new InstantCommand(

                                () -> drivetrain.setReachedTarget(true)));

                NamedCommands.registerCommand("ResetReachedTarget",
                                new InstantCommand(() -> drivetrain.setReachedTarget(false)));

                
                // Configure the trigger bindings
                configureBindings();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
                SmartDashboard.putData("Active Auto Path", autoField);

        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
         * predicate, or via the named factories in
         * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers
         * or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
         */
        private void configureBindings() {

                Driver1.leftStick().onTrue(new InstantCommand(() -> drivetrain.zeroHeading())); // TODO:
                                                                                                // Change
               
                Driver1.x().whileTrue(new CMD_PathfindReefAlign(drivetrain, photonVision, true,
                                () -> targetId, () -> listIndex));
                Driver1.b().whileTrue(new CMD_PathfindReefAlign(drivetrain, photonVision, false,
                                () -> targetId, () -> listIndex));

                Driver1.leftBumper().whileTrue(
                                new CMD_OldPathfindReefAlign(drivetrain, photonVision, true)); // Right
                Driver1.leftTrigger().whileTrue(
                                new CMD_OldPathfindReefAlign(drivetrain, photonVision, false)); // Left

                Driver1.rightStick().onTrue(Commands.none())
                                .onFalse(new InstantCommand(() -> getSelectedReefSide()));

                Driver1.povLeft()
                                .whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-deadbandCompensate(Driver1.getLeftY()) * TunerConstants.kSpeedAt12VoltsMps)
                                        .withVelocityY(-deadbandCompensate(Driver1.getLeftX()) * TunerConstants.kSpeedAt12VoltsMps)
                                        .withRotationalRate(0)));
                Driver1.povUpLeft()
                                .whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-deadbandCompensate(Driver1.getLeftY()) * TunerConstants.kSpeedAt12VoltsMps)
                                        .withVelocityY(-deadbandCompensate(Driver1.getLeftX()) * TunerConstants.kSpeedAt12VoltsMps)
                                        .withRotationalRate(0)));
                Driver1.povDownLeft()
                                .whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-deadbandCompensate(Driver1.getLeftY()) * TunerConstants.kSpeedAt12VoltsMps)
                                        .withVelocityY(-deadbandCompensate(Driver1.getLeftX()) * TunerConstants.kSpeedAt12VoltsMps)
                                        .withRotationalRate(0)));

                // Driver1.rightStick();
                // Driver 2

                // Driver2.a().onTrue(new
                // InstantCommand(()->pivot.changeSetpoint(PivotConstants.kIntakeSetpoint)));
                // Driver2.b().onTrue(new
                // InstantCommand(()->pivot.changeSetpoint(PivotConstants.kL2Setpoint)));
                // Driver2.x().onTrue(new
                // InstantCommand(()->pivot.changeSetpoint(PivotConstants.kL3Setpoint)));
                // Driver2.y().onTrue(new
                // InstantCommand(()->pivot.changeSetpoint(PivotConstants.kL4Setpoint)));

    
                // Driver2.povRight().onTrue(getBargeScoringCommand());



                // Driver2.povDown().onTrue(new InstantCommand(() ->
                // pivot.changeVoltage(-0.02)));
                // Driver2.povUp().onTrue(new InstantCommand(() -> pivot.changeVoltage(0.02)));

                // Driver2.leftBumper()
                // .whileTrue(new InstantCommand(() -> roller.timerInteract(true))
                // .andThen(new RunCommand(
                // () -> roller.setRollerOutput(Roller.kIntakeSpeed),
                // roller).until(() -> roller.atCurrentThresholdandTimerElapsed()))
                // .andThen(new ParallelCommandGroup(
                // new RunCommand(() -> roller.setRollerOutput(
                // Roller.kIntakeFinishSpeed), roller),
                // new InstantCommand(
                // () -> roller.timerInteract(false)),
                // new InstantCommand(() -> Driver1.getHID().setRumble(
                // RumbleType.kBothRumble, 1)),
                // new InstantCommand(() -> Driver2.getHID().setRumble(
                // RumbleType.kBothRumble, 1)),
                // new InstantCommand(() -> roller.hasCoral(true)))
                // .withTimeout(Roller.kIntakeFinishTime)
                // .andThen(new ParallelCommandGroup(
                // new InstantCommand(
                // () -> Driver1.getHID()
                // .setRumble(RumbleType.kBothRumble,
                // 0)),
                // new InstantCommand(
                // () -> Driver2.getHID()
                // .setRumble(RumbleType.kBothRumble,
                // 0)))))
                // .andThen(new InstantCommand(() -> roller.setRollerOutput(0.),
                // roller)))
                // .onFalse(new InstantCommand(() -> roller.setRollerOutput(0.),
                // roller));

                // Driver2.leftBumper()
                // .whileTrue(new RunCommand(() -> roller.setRollerOutput(-Roller.kIntakeSpeed),
                // roller)
                // .andThen(Commands.waitSeconds(1)).andThen(new InstantCommand(() -> pivot
                // .changeSetpoint(PivotConstants.kElevatingSetpoint))))
                // .onFalse(new InstantCommand(() -> roller.setRollerOutput(0), roller));

                // Driver2.leftBumper().whileTrue(new InstantCommand(() -> pivot
                // .changeSetpoint(PivotConstants.kElevatingSetpoint)).alongWith(
                // new RunCommand(() -> roller.setRollerOutput(
                // -Roller.kIntakeSpeed))))
                // .onFalse(new InstantCommand(() -> roller.setRollerOutput(0),
                // roller));

        }

        public double deadbandCompensate(double axis){
                if (Math.abs(axis) < .1){
                        return 0.0;
                }
                else{
                        return Math.copySign((Math.abs(axis) - .1) * (1/0.9), axis);
                }
        }

        public void robotInit() {
                Pathfinding.setPathfinder(new LocalADStar());
                powerDistribution.setSwitchableChannel(true);
        }

        public void getSelectedReefSide() {
                double x = Driver1.getRawAxis(4);
                double y = -Driver1.getRawAxis(5);
                int[] targetTagSet = DriverStation.getAlliance().equals(Optional.of(Alliance.Red))
                                ? new int[] {10, 11, 6, 7, 8, 9}
                                : new int[] {21, 20, 19, 18, 17, 22};
                double angleRadians;
                if (x == 0 && y == 0) {
                        angleRadians = 0.0;
                } else {
                        angleRadians = Math.atan2(y, x) - (Math.PI / 2);
                }
                double angleDegrees = angleRadians * 180 / Math.PI;
                int reefAngleDegrees = (int) Math.round((angleDegrees) / 60) * 60;
                listIndex = Math.floorMod((int) Math.round((angleDegrees) / 60), 6);
                // long listIndex = Math.round((angleDegrees)/60);

                SmartDashboard.putNumber("Angle", angleDegrees);
                SmartDashboard.putNumber("Reef Side Angle", reefAngleDegrees);
                SmartDashboard.putNumber("Reef Align Target ID", targetTagSet[listIndex]);

                Pose2d pose = photonVision.at_field.getTagPose(targetId).orElse(new Pose3d())
                                .toPose2d();
                drivetrain.publisher1.set(pose);
                targetId = targetTagSet[listIndex];

        }

        public Command getPathCommand(String pathName) {
                Pathfinding.setPathfinder(new LocalADStar());
                try {
                        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                        PathConstraints constraints = new PathConstraints(0.5, 0.5,
                                        Units.degreesToRadians(180), Units.degreesToRadians(180)); // unstable
                        return AutoBuilder.pathfindThenFollowPath(path, constraints);
                } catch (Exception e) {
                        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                        return Commands.none();
                }
        }

        

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
                // Pathfinding.setPathfinder(new LocalADStar());

                // try{
                // // Load the path we want to pathfind to and follow
                // PathPlannerPath path = PathPlannerPath.fromPathFile("New Path");
                // drivetrain.publisher1.set(path.getStartingHolonomicPose().get());
                // // // Create the constraints to use while pathfinding. The constraints
                // defined in the path will only be used for the path.
                // PathConstraints constraints = new PathConstraints(
                // 0.5, 0.5,
                // Units.degreesToRadians(180), Units.degreesToRadians(180));

                // // Since AutoBuilder is configured, we can use it to build pathfinding
                // commands
                // return AutoBuilder.pathfindThenFollowPath(
                // path,
                // constraints);
                // return AutoBuilder.followPath(path);

                // PathPlannerAuto auto = new PathPlannerAuto("Cage 4 - E (L4) - C (L4)");
                // return auto;
                // drivetrain.resetPose(new Pose2d(2.0, 3.0, new Rotation2d(Math.toRadians(90))));
                // return new CMD_PathfindReefAlign(drivetrain, photonVision, false, 6, 2);

                // RobotConfig robotConfig = RobotConfig.fromGUISettings();
                // PathPlannerTrajectory traj = path.getIdealTrajectory(robotConfig).get();

                // drivetrain.resetPose(
                // AllianceFlipUtil.apply(path.getStartingHolonomicPose().get())
                // );
                // return AutoBuilder.followPath(path);
                // } catch (Exception e) {
                // DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                // return Commands.none();
                // }

        }

        public Command constructAligningCommand(boolean isLeftAlign) {
                Pose2d tagPose = new Pose2d();
                Integer targetId = 7;
                double xMagnitude = Constants.Drivetrain.kXShiftMagnitude;
                double yMagnitude = Constants.Drivetrain.kYShiftMagnitude;

                List<Integer> targetTagSet;
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                        targetTagSet = alliance.get() == DriverStation.Alliance.Red
                                        ? Arrays.asList(7, 8, 9, 10, 11, 6)
                                        : Arrays.asList(21, 20, 19, 18, 17, 22);
                } else {
                        return null;
                }

                double minDistance = Double.MAX_VALUE;
                for (int tag : targetTagSet) {
                        Pose2d pose = photonVision.at_field.getTagPose(tag).orElse(new Pose3d())
                                        .toPose2d();
                        Translation2d translate = pose.minus(drivetrain.getPose()).getTranslation();
                        double distance = translate.getNorm();

                        if (distance < minDistance) {
                                tagPose = pose;
                                targetId = tag;
                                minDistance = distance;
                        }
                }

                double angle = Units.degreesToRadians(60 * targetTagSet.indexOf(targetId));
                double offset = Units.degreesToRadians(isLeftAlign ? 90 : -90);

                double x = xMagnitude * Math.cos(angle) + yMagnitude * Math.cos(angle + offset);
                double y = xMagnitude * Math.sin(angle) + yMagnitude * Math.sin(angle + offset);

                PathConstraints constraints = new PathConstraints(3.0, 4.0,
                                Units.degreesToRadians(540), Units.degreesToRadians(720));
                return AutoBuilder.pathfindToPose(
                                new Pose2d(tagPose.getX() + x, tagPose.getY() + y,
                                                tagPose.getRotation()
                                                                .plus(Rotation2d.fromRadians(
                                                                                Math.PI / 2.0))),
                                constraints);
        }

        public void robotPeriodic() {

                SmartDashboard.putNumber("Battery Voltage", powerDistribution.getVoltage());
                SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
                autoField.setRobotPose(drivetrain.getPose());
        }

        public void autonomousInit() {
                drivetrain.setIntakeComplete(true);
                drivetrain.setReachedTarget(false);
                Elastic.selectTab("Autonomous");
                PathPlannerLogging.setLogTargetPoseCallback((pose) -> {

                        Pose2d currentPose = drivetrain.getPose();

                        SmartDashboard.putNumber("X Error", pose.getX() - currentPose.getX());
                        SmartDashboard.putNumber("Y Error", pose.getY() - currentPose.getY());
                        SmartDashboard.putNumber("Theta Error", pose.getRotation().getRadians()
                                        - currentPose.getRotation().getRadians());
                        SmartDashboard.putNumber("Desired Theta", pose.getRotation().getRadians());
                        SmartDashboard.putNumber("Actual Theta",
                                        currentPose.getRotation().getRadians());

                });
        }

        public void autonomousPeriodic() {
                photonPoseUpdate();
        }

        public void teleopInit() {
                Elastic.selectTab("Teleoperated");
        }

        public void teleopPeriodic() {
                photonPoseUpdate();
        }

        public void disabledPeriodic() {
                newAutoName = getAutonomousCommand().getName();
                alliance = DriverStation.getAlliance();
                if (autoName != newAutoName || alliance != lastAlliance) {
                        autoName = newAutoName;
                        lastAlliance = alliance;
                        if (AutoBuilder.getAllAutoNames().contains(autoName)) {
                                try {
                                        List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto
                                                        .getPathGroupFromAutoFile(autoName);
                                        List<Pose2d> poses = new ArrayList<>();
                                        for (PathPlannerPath path : pathPlannerPaths) {

                                                if (DriverStation.getAlliance().equals(
                                                                Optional.of(Alliance.Red))) {
                                                        poses.addAll(path.getAllPathPoints()
                                                                        .stream()
                                                                        .map(point -> new Pose2d(
                                                                                        Field.fieldLength
                                                                                                        - point.position.getX(),
                                                                                        Field.fieldWidth - point.position
                                                                                                        .getY(),
                                                                                        new Rotation2d()))
                                                                        .collect(Collectors
                                                                                        .toList()));
                                                } else {
                                                        poses.addAll(path.getAllPathPoints()
                                                                        .stream()
                                                                        .map(point -> new Pose2d(
                                                                                        point.position.getX(),
                                                                                        point.position.getY(),
                                                                                        new Rotation2d()))
                                                                        .collect(Collectors
                                                                                        .toList()));
                                                }
                                        }
                                        autoField.getObject("path").setPoses(poses);
                                } catch (IOException e) {
                                        e.printStackTrace();
                                        return;
                                } catch (ParseException e) {
                                        e.printStackTrace();
                                        return;
                                }
                        }
                }
                photonPoseUpdate();
        }

        public static void photonPoseUpdate() {
                Optional<EstimatedRobotPose> photonPoseOptional = photonVision.getCam1Pose();

                if (photonPoseOptional.isPresent()) {
                        Pose3d photonPose = photonPoseOptional.get().estimatedPose;
                        double odometryDifference = (drivetrain.getPose().minus(new Pose2d(photonPose.getX(),photonPose.getY(), new Rotation2d(0)))).getTranslation().getNorm();
                        if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
                                        && photonPose.getY() >= 0
                                        && photonPose.getY() <= Field.fieldWidth
                                        && photonVision.getCam1BestTarget() != null
                                        // && odometryDifference > 2
                                        ) {

                                Pose2d closestTag = photonVision.at_field.getTagPose(
                                                photonVision.getCam1BestTarget().getFiducialId())
                                                .get().toPose2d();
                                Translation2d translate = closestTag.minus(photonPose.toPose2d())
                                                .getTranslation();

                                double distance = translate.getNorm();
                                double xStddev = Math.pow(distance, 2) / (8.0088 * 0.5);
                                double yStddev = xStddev;
                                double rotStddev = Units.degreesToRadians(120.0);
                                drivetrain.publisher3.set(photonPose.toPose2d());
                                drivetrain.setVisionMeasurementStdDevs(
                                                VecBuilder.fill(xStddev, yStddev, rotStddev));
                                drivetrain.addVisionMeasurement(photonPose.toPose2d(),
                                                photonPoseOptional.get().timestampSeconds);
                                drivetrain.publisher3.set(photonPose.toPose2d());
                                SmartDashboard.putNumber("Cam 1 Closest Tag",
                                                photonVision.getCam1BestTarget().getFiducialId());
                        }
                }

                photonPoseOptional = photonVision.getCam2Pose();

                if (photonPoseOptional.isPresent()) {
                        Pose3d photonPose = photonPoseOptional.get().estimatedPose;
                        double odometryDifference = (drivetrain.getPose().minus(new Pose2d(photonPose.getX(),photonPose.getY(), new Rotation2d(0)))).getTranslation().getNorm();
                        if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
                                        && photonPose.getY() >= 0
                                        && photonPose.getY() <= Field.fieldWidth
                                        && photonVision.getCam2BestTarget() != null
                                        // && odometryDifference > 2
                                        ) {

                                Pose2d closestTag = photonVision.at_field.getTagPose(
                                                photonVision.getCam2BestTarget().getFiducialId())
                                                .get().toPose2d();
                                Translation2d translate = closestTag.minus(photonPose.toPose2d())
                                                .getTranslation();

                                double distance = translate.getNorm();
                                double xStddev = Math.pow(distance, 2) / 8.0088;
                                double yStddev = xStddev;
                                double rotStddev = Units.degreesToRadians(120.0);
                                drivetrain.publisher4.set(photonPose.toPose2d());
                                drivetrain.setVisionMeasurementStdDevs(
                                                VecBuilder.fill(xStddev, yStddev, rotStddev));
                                drivetrain.addVisionMeasurement(photonPose.toPose2d(),
                                                photonPoseOptional.get().timestampSeconds);

                                drivetrain.publisher4.set(photonPose.toPose2d());
                                SmartDashboard.putNumber("Cam 2 Closest Tag",
                                                photonVision.getCam2BestTarget().getFiducialId());
                        }
                }
        }

        // public static void photonAutonPoseUpdate() {
        //         Optional<EstimatedRobotPose> photonPoseOptional = photonVision.getCam1Pose();

        //         if (photonPoseOptional.isPresent()) {
        //                 Pose3d photonPose = photonPoseOptional.get().estimatedPose;

        //                 if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
        //                                 && photonPose.getY() >= 0
        //                                 && photonPose.getY() <= Field.fieldWidth
        //                                 && photonVision.getCam1BestTarget() != null) {

        //                         Pose2d closestTag = photonVision.at_field.getTagPose(
        //                                         photonVision.getCam1BestTarget().getFiducialId())
        //                                         .get().toPose2d();
        //                         Translation2d translate = closestTag.minus(photonPose.toPose2d())
        //                                         .getTranslation();

        //                         double distance = translate.getNorm();
        //                         double xStddev = Math.pow(distance, 1.75) * (3 * (Math.sqrt(Math
        //                                         .pow(drivetrain.getChassisSpeeds().vxMetersPerSecond,
        //                                                         2)
        //                                         + Math.pow(drivetrain
        //                                                         .getChassisSpeeds().vyMetersPerSecond,
        //                                                         2)))
        //                                         / 4.92 + 2) / 3.6;
        //                         double yStddev = xStddev;
        //                         double rotStddev = Units.degreesToRadians(120.0);
        //                         drivetrain.publisher3.set(photonPose.toPose2d());
        //                         drivetrain.setVisionMeasurementStdDevs(
        //                                         VecBuilder.fill(xStddev, yStddev, rotStddev));
        //                         drivetrain.addVisionMeasurement(photonPose.toPose2d(),
        //                                         photonPoseOptional.get().timestampSeconds);
        //                         drivetrain.publisher3.set(photonPose.toPose2d());
        //                         SmartDashboard.putNumber("Cam 1 Closest Tag",
        //                                         photonVision.getCam1BestTarget().getFiducialId());
        //                 }
        //         }

        //         photonPoseOptional = photonVision.getCam2Pose();

        //         if (photonPoseOptional.isPresent()) {
        //                 Pose3d photonPose = photonPoseOptional.get().estimatedPose;

        //                 if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
        //                                 && photonPose.getY() >= 0
        //                                 && photonPose.getY() <= Field.fieldWidth
        //                                 && photonVision.getCam2BestTarget() != null) {

        //                         Pose2d closestTag = photonVision.at_field.getTagPose(
        //                                         photonVision.getCam2BestTarget().getFiducialId())
        //                                         .get().toPose2d();
        //                         Translation2d translate = closestTag.minus(photonPose.toPose2d())
        //                                         .getTranslation();

        //                         double distance = translate.getNorm();
        //                         double xStddev = Math.pow(distance, 1.75) * (3 * (Math.sqrt(Math
        //                                         .pow(drivetrain.getChassisSpeeds().vxMetersPerSecond,
        //                                                         2)
        //                                         + Math.pow(drivetrain
        //                                                         .getChassisSpeeds().vyMetersPerSecond,
        //                                                         2)))
        //                                         / 4.92 + 2) / 3.6;
        //                         double yStddev = xStddev;
        //                         double rotStddev = Units.degreesToRadians(120.0);
// drivetrain.publisher4.set(photonPose.toPose2d());
        //                         drivetrain.setVisionMeasurementStdDevs(
        //                                         VecBuilder.fill(xStddev, yStddev, rotStddev));
        //                         drivetrain.addVisionMeasurement(photonPose.toPose2d(),
        //                                         photonPoseOptional.get().timestampSeconds);

// drivetrain.publisher4.set(photonPose.toPose2d());
        //                         SmartDashboard.putNumber("Cam 2 Closest Tag",
        //                         photonVision.getCam2BestTarget().getFiducialId());
        //                 }
        //         }
        // }

        // public static void photonDisabledPoseUpdate() {
        //         Optional<EstimatedRobotPose> photonPoseOptional = photonVision.getCam1Pose();

        //         if (photonPoseOptional.isPresent()) {
        //                 Pose3d photonPose = photonPoseOptional.get().estimatedPose;

        //                 if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
        //                                 && photonPose.getY() >= 0
        //                                 && photonPose.getY() <= Field.fieldWidth
        //                                 && photonVision.getCam1BestTarget() != null) {

        //                         Pose2d closestTag = photonVision.at_field.getTagPose(
        //                                         photonVision.getCam1BestTarget().getFiducialId())
        //                                         .get().toPose2d();
        //                         Translation2d translate = closestTag.minus(photonPose.toPose2d())
        //                                         .getTranslation();

        //                         double distance = translate.getNorm();
        //                         double xStddev = Math.pow(distance, 2) / 8.0088;
        //                         double yStddev = xStddev;
        //                         double rotStddev = Units.degreesToRadians(120.0);
        //                         drivetrain.publisher3.set(photonPose.toPose2d());
        //                         drivetrain.setVisionMeasurementStdDevs(
        //                                         VecBuilder.fill(xStddev, yStddev, rotStddev));
        //                         drivetrain.addVisionMeasurement(photonPose.toPose2d(),
        //                                         photonPoseOptional.get().timestampSeconds);
        //                         drivetrain.publisher3.set(photonPose.toPose2d());
        //                         SmartDashboard.putNumber("Cam 1 Closest Tag",
        //                                         photonVision.getCam1BestTarget().getFiducialId());
        //                 }
        //         }

        //         photonPoseOptional = photonVision.getCam2Pose();

        //         if (photonPoseOptional.isPresent()) {
        //                 Pose3d photonPose = photonPoseOptional.get().estimatedPose;

        //                 if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
        //                                 && photonPose.getY() >= 0
        //                                 && photonPose.getY() <= Field.fieldWidth
        //                                 && photonVision.getCam2BestTarget() != null) {

        //                         Pose2d closestTag = photonVision.at_field.getTagPose(
        //                                         photonVision.getCam2BestTarget().getFiducialId())
        //                                         .get().toPose2d();
        //                         Translation2d translate = closestTag.minus(photonPose.toPose2d())
        //                                         .getTranslation();

        //                         double distance = translate.getNorm();
        //                         double xStddev = Math.pow(distance, 2) / 8.0088;
        //                         double yStddev = xStddev;
        //                         double rotStddev = Units.degreesToRadians(120.0);
// drivetrain.publisher4.set(photonPose.toPose2d());
        //                         drivetrain.setVisionMeasurementStdDevs(
        //                                         VecBuilder.fill(xStddev, yStddev, rotStddev));
        //                         drivetrain.addVisionMeasurement(photonPose.toPose2d(),
        //                                         photonPoseOptional.get().timestampSeconds);

// drivetrain.publisher4.set(photonPose.toPose2d());
        //                         SmartDashboard.putNumber("Cam 2 Closest Tag",
        //                         photonVision.getCam2BestTarget().getFiducialId());
        //                 }
        //         }
        // }
}
