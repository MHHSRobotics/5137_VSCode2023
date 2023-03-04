package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Drive_Constants;

public class Auto_Subsystem extends SubsystemBase {
    private final Drive_Subsystem drive;
    private final RamseteAutoBuilder autoBuilder; //Allows auto to drive a path
    private final HashMap<String, Command> eventMap; //Maps out events during autoPath

    private final ArrayList<PathPlannerTrajectory> left_mobility;
    private final ArrayList<PathPlannerTrajectory> left_engage;
    private final ArrayList<PathPlannerTrajectory> left_mobility_engage;
    private final ArrayList<PathPlannerTrajectory> left_doubleScore;
    private final ArrayList<PathPlannerTrajectory> left_doubleScore_engage;
    private final ArrayList<PathPlannerTrajectory> middle_engage;
    private final ArrayList<PathPlannerTrajectory> right_mobility;
    private final ArrayList<PathPlannerTrajectory> right_engage;
    private final ArrayList<PathPlannerTrajectory> right_mobility_engage;
    private final ArrayList<PathPlannerTrajectory> right_doubleScore;
    private final ArrayList<PathPlannerTrajectory> right_doubleScore_engage;

    public Auto_Subsystem(Drive_Subsystem drive) {
        this.drive = drive;

        eventMap = new HashMap<>();

        autoBuilder = new RamseteAutoBuilder(
        drive::getPose, // Pose2d supplier method from drivetrain
        drive::resetPose, // Pose2d consume method used to reset odometry at the beginning of auto
        new RamseteController(),
        new DifferentialDriveKinematics(Units.inchesToMeters(Drive_Constants.trackWidth)), //Kinematics for our drivebase
        drive.voltPID,
        drive::getWheelSpeeds,
        Drive_Constants.drivePIDConstants,
        drive::setVolts,
        eventMap, //Event map that maps out commands to specific keywords in autoPath markers
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        drive); // The drive subsystem. Used to properly set the requirements of path following commands

        //Loads created paths
        left_mobility = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("leftPos_mobility", new PathConstraints(4, 3));
        left_engage = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("leftPos_engage", new PathConstraints(4, 3));
        left_mobility_engage = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("leftPos_mobility_engage", new PathConstraints(4, 3));
        left_doubleScore =  (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("leftPos_doubleScore", new PathConstraints(4, 3));
        left_doubleScore_engage = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("leftPos_doubleScore_engage", new PathConstraints(4, 3));
        middle_engage = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("middlePos_engage", new PathConstraints(4, 3));
        right_mobility = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("rightPos_mobility", new PathConstraints(4, 3));
        right_engage = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("rightPos_engage", new PathConstraints(4, 3));
        right_mobility_engage = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("rightPos_mobility_engage", new PathConstraints(4, 3));
        right_doubleScore =  (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("rightPos_doubleScore", new PathConstraints(4, 3));
        right_doubleScore_engage = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("rightPos_doubleScore_engage", new PathConstraints(4, 3));
    }
}