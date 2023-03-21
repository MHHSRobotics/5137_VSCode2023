package frc.robot.systems;

import java.util.ArrayList;
import java.util.HashMap;

import javax.sound.midi.Sequence;

import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Drive_Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive_Commands;
import frc.robot.commands.Punch_Commands;
import frc.robot.constants.Auto_Constants;
import frc.robot.subsystems.Drive_Subsystem;
import frc.robot.subsystems.Punch_Subystem;


public class AutoManager extends SubsystemBase {
    private  Drive_Subsystem drive;
    
    private  double maxVelo;
    private  double maxAccel;

    public  SequentialCommandGroup autoPunch;
    public  SequentialCommandGroup scoreTopCube;
    public  SequentialCommandGroup intakeObject;

    private  HashMap<String, Command> eventMap; //Maps out events during autoPath
    private  RamseteAutoBuilder autoBuilder; //Allows auto to drive a path
    
    private  ArrayList<PathPlannerTrajectory> left_mobility;
    private  ArrayList<PathPlannerTrajectory> middle_engage;
    private  ArrayList<PathPlannerTrajectory> right_mobility;
    private  ArrayList<PathPlannerTrajectory> middle_mobility_engage;

    public SequentialCommandGroup timedMobility = new SequentialCommandGroup(RobotContainer.punch_Commands.Punch(), RobotContainer.drive_Commands.timedDrive(1.85, 1));

    public SequentialCommandGroup timedEngage = new SequentialCommandGroup(RobotContainer.punch_Commands.Punch(), RobotContainer.drive_Commands.timedDrive(1.05, 1), RobotContainer.drive_Commands.balance(), RobotContainer.drive_Commands.setBrake(true));

    public SequentialCommandGroup timedMobilityEngage = new SequentialCommandGroup(RobotContainer.punch_Commands.Punch(), RobotContainer.drive_Commands.timedDrive(1.05, 1),RobotContainer.drive_Commands.timedDrive(2.4, .4),RobotContainer.drive_Commands.timedDrive(1.4, -1), RobotContainer.drive_Commands.balance(), RobotContainer.drive_Commands.setBrake(true));


    public AutoManager(Drive_Subsystem drive, Punch_Commands punch) {
        this.drive = drive;


        maxVelo = Auto_Constants.maxVelo;
        maxAccel = Auto_Constants.maxAccel;

        autoPunch = new SequentialCommandGroup(punch.Punch());

        eventMap = new HashMap<>();



        autoBuilder = new RamseteAutoBuilder(
            drive::getPose,
             drive::resetPose,
              new RamseteController(),
               Drive_Constants.trackWidth,
                drive::setSpeeds,
                 eventMap,
                  true,
                  drive);

                  /* 
        autoBuilder = new RamseteAutoBuilder(
        drive::getPose, // Pose2d supplier method from drivetrain
        drive::resetPose, // Pose2d consume method used to reset odometry at the beginning of auto
        new RamseteController(),
        Drive_Constants.trackWidth, //Kinematics for our drivebase
        drive.voltPID,
        drive::getWheelSpeeds,
        Drive_Constants.drivePIDConstants,
        drive::setVolts,
        eventMap, //Event map that maps out commands to specific keywords in autoPath markers
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        drive); // The drive subsystem. Used to properly set the requirements of path following commands

        */
        //Loads created paths
        left_mobility = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("leftPos_mobility", new PathConstraints(maxVelo, maxAccel));
        middle_engage = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("middlePos_engage", new PathConstraints(maxVelo, maxAccel));
        right_mobility = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("rightPos_mobility", new PathConstraints(maxVelo, maxAccel));
        middle_mobility_engage = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("rightPos_mobility_engage", new PathConstraints(maxVelo, maxAccel));
    }

    public void runAuto(String choice) {
        ArrayList<PathPlannerTrajectory> autoPath;
        String[] info = choice.split("@");
        String type = info[0];
        String score = info[1];

        System.out.println(type);
        System.out.println(score);

        switch (type) {
            case ("leftMobility"): {autoPath = left_mobility;}
            case ("middleEngage"): {autoPath = middle_engage;}
            case ("rightMobility"): {autoPath = right_mobility;}
            case ("middleCombo"): {autoPath = middle_mobility_engage;}
            default: {autoPath = middle_engage;}
        }

        eventMap.put("Balance", RobotContainer.drive_Commands.balance());
        eventMap.put("Brake", RobotContainer.drive_Commands.setBrake(true));

        if (score.equals("Score")) {
            eventMap.put("ScoreCone", autoPunch);
        }
         
        if (type != "None") {
            autoBuilder.fullAuto(autoPath).schedule();
        }
    }
}