// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  public SwerveDrive swerveDrive = new SwerveDrive();
  //
  public static final Joystick controller = new Joystick(0);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    //var a = new SwerveDrivePath(new Vector2(232.8, 37), new Vector2(0.297, -0.36), new Vector2(294.4, 19.5), new Vector2(-0.10938007131100254, -0.994));
    //System.out.println(a.GetClosestTime(new Vector2(252, 42.7)));
    
    //m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    //m_chooser.addOption("My Auto", kCustomAuto);
    //SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    swerveDrive.Start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Vector2 joystickVector = new Vector2(controller.getRawAxis(0), -controller.getRawAxis(1));
    double rotationSpeed = controller.getRawAxis(4);
    if(controller.getRawButtonReleased(8)) {
      swerveDrive.ResetRotation();
      System.out.println("Reseted Swerve rotation, instructed by controller");
    }
    if(Math.abs(rotationSpeed) < 0.1) {
      rotationSpeed = 0;
    }
    double joyMag = joystickVector.getMagnitude();
    double curvedMag = (3*joyMag*joyMag*joyMag+1)/4*Math.sqrt(joyMag);
    curvedMag = Math.max((curvedMag-Constants.DriveConstants.driveDeadband)/(1-Constants.DriveConstants.driveDeadband), 0);
    double absRotation = Math.abs(rotationSpeed);
    double newRotationSpeed = ((3*absRotation*absRotation*absRotation+1)/4*Math.sqrt(absRotation)) * Math.signum(rotationSpeed); 
    Vector2[] wheelSpeeds = swerveDrive.SetWheelSpeeds(curvedMag, joystickVector.getAngle(), newRotationSpeed);
    //System.out.println(swerveDrive.m_navx.getDisplacementX() + " " + swerveDrive.m_navx.getDisplacementY() + " " + swerveDrive.m_navx.getDisplacementZ());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
