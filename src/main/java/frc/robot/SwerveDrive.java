package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Map;

public class SwerveDrive extends SubsystemBase {
    CANSparkMax  frontLeftDriveMotor = new CANSparkMax(Constants.DriveConstants.swerveMotorDrivePorts[0], MotorType.kBrushless);
    CANSparkMax  frontRightDriveMotor = new CANSparkMax(Constants.DriveConstants.swerveMotorDrivePorts[1], MotorType.kBrushless);
    CANSparkMax  backLeftDriveMotor = new CANSparkMax(Constants.DriveConstants.swerveMotorDrivePorts[2], MotorType.kBrushless);
    CANSparkMax  backRightDriveMotor = new CANSparkMax(Constants.DriveConstants.swerveMotorDrivePorts[3], MotorType.kBrushless);

    CANSparkMax  frontLeftTurnMotor = new CANSparkMax(Constants.DriveConstants.swerveMotorTurnPorts[0], MotorType.kBrushless);
    CANSparkMax  frontRightTurnMotor = new CANSparkMax(Constants.DriveConstants.swerveMotorTurnPorts[1], MotorType.kBrushless);
    CANSparkMax  backLeftTurnMotor = new CANSparkMax(Constants.DriveConstants.swerveMotorTurnPorts[2], MotorType.kBrushless);
    CANSparkMax  backRightTurnMotor = new CANSparkMax(Constants.DriveConstants.swerveMotorTurnPorts[3], MotorType.kBrushless);
    
    CANCoder frontLeftEncoder = new CANCoder(Constants.DriveConstants.swerveEncoderPorts[0]);
    CANCoder frontRightEncoder = new CANCoder(Constants.DriveConstants.swerveEncoderPorts[1]);
    CANCoder backLeftEncoder = new CANCoder(Constants.DriveConstants.swerveEncoderPorts[2]);
    CANCoder backRightEncoder = new CANCoder(Constants.DriveConstants.swerveEncoderPorts[3]);
    
    public boolean fieldOriented = true;

    private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);
    PIDController pid = new PIDController(Constants.DriveConstants.turnPID_P, Constants.DriveConstants.turnPID_I, Constants.DriveConstants.turnPID_D);
    
    
    //Shuffle Board//
    private NetworkTableEntry maxSwerveShuffle(String name) {
        return Shuffleboard.getTab("Drive").add(name, SwerveDriveMathConstants.maxMotorUse_default)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1)).getEntry();
    }
    private NetworkTableEntry motorSwerveShuffle(String name) {
        return Shuffleboard.getTab("Drive").add(name, "<0, 0>")
        .withWidget(BuiltInWidgets.kTextView).getEntry();
    }
    private NetworkTableEntry encoderSwerveShuffle(String name) {
        return Shuffleboard.getTab("Drive").add(name, "N/A°")
        .withWidget(BuiltInWidgets.kTextView).getEntry();
    }
    private NetworkTableEntry maxSwerveMotorUse = maxSwerveShuffle("Max Swerve Motor Use");
    private NetworkTableEntry maxSwerveRotationSpeed = maxSwerveShuffle("Max Swerve Rotation Speed");
    private NetworkTableEntry maxSwerveSpeed = maxSwerveShuffle("Max Swerve Speed");
    private NetworkTableEntry[] swerveWheels = {motorSwerveShuffle("Front Left Wheel"), motorSwerveShuffle("Front Right Wheel"), motorSwerveShuffle("Back Left Wheel"), motorSwerveShuffle("Back Right Wheel")};
    private NetworkTableEntry[] encoders = {encoderSwerveShuffle("Front Left Encoder"), encoderSwerveShuffle("Front Right Encoder"), encoderSwerveShuffle("Back Left Encoder"), encoderSwerveShuffle("Back Right Encoder")};
    /////////////////


    public SwerveDrive() {
        pid.enableContinuousInput(0, 2 * Math.PI);
    }
    double StandardizeRAD(double radians) {
        return (radians + 2 * Math.PI) % (2 * Math.PI);
    }
    public Vector2[] SetWheelSpeeds(double joystickMag, double joystickAngle, double joystickRotation) {    
        double maxMotorUse = maxSwerveMotorUse.getDouble(SwerveDriveMathConstants.maxMotorUse_default);
        double maxRotationSpeed = maxSwerveRotationSpeed.getDouble(SwerveDriveMathConstants.maxRotationSlider_default);
        double maxMoveSpeed = maxSwerveSpeed.getDouble(SwerveDriveMathConstants.maxMoveSpeed_default);
        if(maxMotorUse != SwerveDriveMathConstants.maxMotorUse || maxRotationSpeed != SwerveDriveMathConstants.maxRotationSlider || maxMoveSpeed != SwerveDriveMathConstants.maxMoveSpeed) {
        SwerveDriveMathConstants.maxMotorUse = maxMotorUse;
        SwerveDriveMathConstants.maxRotationSlider = maxRotationSpeed;
        SwerveDriveMathConstants.maxMoveSpeed = maxMoveSpeed;
        SwerveDriveMathConstants.updateMax();
        }
        //
        double robotRotation = 0; // may need to subtract instead of add
        if(fieldOriented) {
            if (m_navx.isMagnetometerCalibrated()) {
                robotRotation = (m_navx.getFusedHeading()) * Math.PI / 180;
            } else {
                robotRotation = -m_navx.getYaw() * Math.PI / 180;
            }
        }
        Vector2[] wheelSpeeds = SwerveDriveMath.WheelSpeeds(Math.min(joystickMag, 1), joystickAngle, joystickRotation, robotRotation);
        frontLeftDriveMotor.set(wheelSpeeds[0].getMagnitude());
        frontRightDriveMotor.set(wheelSpeeds[1].getMagnitude());
        backLeftDriveMotor.set(wheelSpeeds[2].getMagnitude());
        backRightDriveMotor.set(wheelSpeeds[3].getMagnitude());
        frontLeftTurnMotor.set(pid.calculate(StandardizeRAD(frontLeftEncoder.getAbsolutePosition() * Math.PI / 180), wheelSpeeds[0].getAngle()));
        frontRightTurnMotor.set(pid.calculate(StandardizeRAD(frontRightEncoder.getAbsolutePosition() * Math.PI / 180), wheelSpeeds[1].getAngle()));
        backLeftTurnMotor.set(pid.calculate(StandardizeRAD(backLeftEncoder.getAbsolutePosition() * Math.PI / 180), wheelSpeeds[2].getAngle()));
        backRightTurnMotor.set(pid.calculate(StandardizeRAD(backRightEncoder.getAbsolutePosition() * Math.PI / 180), wheelSpeeds[3].getAngle()));
        
        //Shuffle Board//
        for(int i = 0; i < wheelSpeeds.length; i++) {
            swerveWheels[i].setString(wheelSpeeds[i].toString(3));
        }
        encoders[0].setString(frontLeftEncoder.getAbsolutePosition() + "°");
        encoders[1].setString(frontRightEncoder.getAbsolutePosition() + "°");
        encoders[2].setString(backLeftEncoder.getAbsolutePosition() + "°");
        encoders[3].setString(backRightEncoder.getAbsolutePosition() + "°");
        /////////////////
        
        return wheelSpeeds;
    }
}
