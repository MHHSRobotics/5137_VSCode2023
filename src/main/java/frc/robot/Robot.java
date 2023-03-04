// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ControlCommands.UpdateControllerPorts;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer; 
  private Command m_autonomousCommand; //Will be used to run auto
  private int x = 0;
  private int y = 0;
  private boolean m_switch = true;

 
  


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //m_robotContainer.setLedMode();
  
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //runLEDS();
    //m_robotContainer.gradiant();
    m_robotContainer.cardinalgold();
  }

  private void runLEDS() {
    if (x < 250) {
      if (m_switch) {
        m_robotContainer.moveColorsForward();
      } else {
        m_robotContainer.moveColorsBackward();
      }
      x++;
    } else if (y < 30) {
      m_robotContainer.solidCardinal();
      y++;
    } else if (y < 60) {
      m_robotContainer.solidGold();
      y++;
    } else if (y < 90) {
      m_robotContainer.solidCardinal();
      y++;
    } else {
      if (m_switch) {
        m_switch = false;
      } else {
        m_switch = true;
      }
      x = 0;
      y = 0;
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  public void autonomousInit()
  {
    //We should add a command so it will score and then run the path
    m_autonomousCommand = m_robotContainer.getAutoCommand(); //Gets the auto command selected 
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule(); //Runs auto 
    }
  }
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.setLedMode();
    
    //Stops auto command when teleop starts
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Command updateControls = new UpdateControllerPorts();
    updateControls.schedule();
    m_robotContainer.configureBindings();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
