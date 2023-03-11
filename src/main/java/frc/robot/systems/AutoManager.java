package frc.robot.systems;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Drive_Constants;
import frc.robot.constants.Auto_Constants;
import frc.robot.objects.AutoData;
import frc.robot.subsystems.Drive_Subsystem;

import frc.robot.commands.Intake_Commands;
import frc.robot.commands.Arm_Commands;
import frc.robot.commands.Clamp_Commands;

public class AutoManager extends SubsystemBase {
    private  Drive_Subsystem drive;
    private  Intake_Commands intake_Commands;
    private  Arm_Commands arm_Commands;
    private  Clamp_Commands clamp_Commands;
    
    private  double maxVelo;
    private  double maxAccel;

    private  SequentialCommandGroup scoreTopCone;
    private  SequentialCommandGroup scoreTopCube;
    private  SequentialCommandGroup intakeObject;

    private  HashMap<String, Command> eventMap; //Maps out events during autoPath
    private  RamseteAutoBuilder autoBuilder; //Allows auto to drive a path
    
    private  ArrayList<PathPlannerTrajectory> left_mobility;
    private  ArrayList<PathPlannerTrajectory> left_engage;
    private  ArrayList<PathPlannerTrajectory> left_mobility_engage;
    private  ArrayList<PathPlannerTrajectory> left_doubleScore;
    private  ArrayList<PathPlannerTrajectory> left_doubleScore_engage;
    private  ArrayList<PathPlannerTrajectory> middle_engage;
    private  ArrayList<PathPlannerTrajectory> right_mobility;
    private  ArrayList<PathPlannerTrajectory> right_engage;
    private  ArrayList<PathPlannerTrajectory> right_mobility_engage;
    private  ArrayList<PathPlannerTrajectory> right_doubleScore;
    private  ArrayList<PathPlannerTrajectory> right_doubleScore_engage;

    

    

    public AutoManager(Drive_Subsystem drive, Intake_Commands intake_Commands, Arm_Commands arm_Commands, Clamp_Commands clamp_Commands) {
        this.drive = drive;

        this.intake_Commands = intake_Commands;
        this.arm_Commands = arm_Commands;
        this.clamp_Commands = clamp_Commands;

        maxVelo = Auto_Constants.maxVelo;
        maxAccel = Auto_Constants.maxAccel;

        scoreTopCone = new SequentialCommandGroup(clamp_Commands.clamp(), arm_Commands.moveToTopCone(), clamp_Commands.clampRelease());
        scoreTopCube = new SequentialCommandGroup(clamp_Commands.clamp(), arm_Commands.moveToTopCube(), clamp_Commands.clampRelease());
        intakeObject = new SequentialCommandGroup(intake_Commands.runIntakeForward(), arm_Commands.moveToIntake());

        eventMap = new HashMap<>();

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

        //Loads created paths
        left_mobility = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("leftPos_mobility", new PathConstraints(maxVelo, maxAccel));
        left_engage = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("leftPos_engage", new PathConstraints(maxVelo, maxAccel));
        left_mobility_engage = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("leftPos_mobility_engage", new PathConstraints(maxVelo, maxAccel));
        left_doubleScore =  (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("leftPos_doubleScore", new PathConstraints(maxVelo, maxAccel));
        left_doubleScore_engage = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("leftPos_doubleScore_engage", new PathConstraints(maxVelo, maxAccel));
        middle_engage = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("middlePos_engage", new PathConstraints(maxVelo, maxAccel));
        right_mobility = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("rightPos_mobility", new PathConstraints(maxVelo, maxAccel));
        right_engage = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("rightPos_engage", new PathConstraints(maxVelo, maxAccel));
        right_mobility_engage = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("rightPos_mobility_engage", new PathConstraints(maxVelo, maxAccel));
        right_doubleScore =  (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("rightPos_doubleScore", new PathConstraints(maxVelo, maxAccel));
        right_doubleScore_engage = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("rightPos_doubleScore_engage", new PathConstraints(maxVelo, maxAccel));
    }

    public void runAuto(AutoData autoInfo) {
        String tempType;
        int autoNumber = 0;
        ArrayList<PathPlannerTrajectory> autoPath;

        if (autoInfo.getType().equals("SingleScore")) {
            tempType = "None";
        } else {
            tempType = autoInfo.getType();
        }

        AutoData tempAuto = new AutoData(autoInfo.getPosition(), tempType, autoInfo.getMobility(), autoInfo.getEngage());

        System.out.println(tempAuto.getPosition());
        System.out.println(tempAuto.getType());
        System.out.println(tempAuto.getMobility());
        System.out.println(tempAuto.getEngage());

        for (int i = 0; i < Auto_Constants.autoAmount; i++) {
            if (tempAuto.equalTo(Auto_Constants.autoChoices[i])) {
                autoNumber = i+1;
            }
        }
        switch (autoNumber) {
            case (1): {autoPath = left_mobility;}
            case (2): {autoPath = left_engage;}
            case (3): {autoPath = left_mobility_engage;}
            case (4): {autoPath = left_doubleScore;}
            case (5): {autoPath = left_doubleScore_engage;}
            case (6): {autoPath = middle_engage;}
            case (7): {autoPath = right_mobility;}
            case (8): {autoPath = right_engage;}
            case (9): {autoPath = right_mobility_engage;}
            case (10): {autoPath = right_doubleScore;}
            case (11): {autoPath = right_doubleScore_engage;}
            default: {autoPath = middle_engage;}
        }

        System.out.println("Auto Selected: "+autoNumber);

        /* 
        if (autoInfo.getType() == "SingleScore")
            eventMap.put("ScoreCone", scoreTopCone);
        else if (autoInfo.getType() == "DoubleScore") {
            eventMap.put("ScoreCone", scoreTopCone);
            eventMap.put("Intake", intakeObject);
            eventMap.put("ScoreCube", scoreTopCube);
        }
        */
        autoBuilder.fullAuto(autoPath).schedule();
    }
}