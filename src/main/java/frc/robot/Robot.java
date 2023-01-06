// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  //Swerve Drive code
  private NetworkTableEntry maxSwerveShuffle(String name) {
    return Shuffleboard.getTab("Drive").add(name, SwerveDriveMathConstants.maxMotorUse_default)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1)).getEntry();
  }
  private NetworkTableEntry motorSwerveShuffle(String name) {
    return Shuffleboard.getTab("Drive").add(name, "<0, 0>")
    .withWidget(BuiltInWidgets.kTextView).getEntry();
  }
  private NetworkTableEntry maxSwerveMotorUse = maxSwerveShuffle("Max Swerve Motor Use");
  private NetworkTableEntry maxSwerveRotationSpeed = maxSwerveShuffle("Max Swerve Rotation Speed");
  private NetworkTableEntry maxSwerveSpeed = maxSwerveShuffle("Max Swerve Speed");
  private NetworkTableEntry[] swerveWheels = {motorSwerveShuffle("Front Left Wheel"), motorSwerveShuffle("Front Right Wheel"), motorSwerveShuffle("Back Left Wheel"), motorSwerveShuffle("Back Right Wheel")};
  
  public SwerveDrive swerveDrive = new SwerveDrive();
  //
  public static final Joystick controller = new Joystick(0);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
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
    Vector2 joystickVector = new Vector2(controller.getRawAxis(4), -controller.getRawAxis(5));
    double rotationSpeed = controller.getRawAxis(0);
    if(Math.abs(joystickVector.x) < 0.1) {
      joystickVector.x = 0;
    }
    if(Math.abs(joystickVector.y) < 0.1) {
      joystickVector.y = 0;
    }
    if(Math.abs(rotationSpeed) < 0.1) {
      rotationSpeed = 0;
    }

    double maxMotorUse = maxSwerveMotorUse.getDouble(SwerveDriveMathConstants.maxMotorUse_default);
    double maxRotationSpeed = maxSwerveRotationSpeed.getDouble(SwerveDriveMathConstants.maxRotationSlider_default);
    double maxMoveSpeed = maxSwerveSpeed.getDouble(SwerveDriveMathConstants.maxMoveSpeed_default);
    if(maxMotorUse != SwerveDriveMathConstants.maxMotorUse || maxRotationSpeed != SwerveDriveMathConstants.maxRotationSlider || maxMoveSpeed != SwerveDriveMathConstants.maxMoveSpeed) {
      SwerveDriveMathConstants.maxMotorUse = maxMotorUse;
      SwerveDriveMathConstants.maxRotationSlider = maxRotationSpeed;
      SwerveDriveMathConstants.maxMoveSpeed = maxMoveSpeed;
      SwerveDriveMathConstants.updateMax();
    }
    Vector2[] wheelSpeeds = swerveDrive.SetWheelSpeeds(joystickVector.getMagnitude(), joystickVector.getAngle(), rotationSpeed);
    for(int i = 0; i < wheelSpeeds.length; i++) {
      swerveWheels[i].setString(wheelSpeeds[i].toString(3));
    }
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
