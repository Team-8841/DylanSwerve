package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.SPI;

import java.text.DecimalFormat;
import java.util.Map;

public class SwerveDrive extends SubsystemBase {
    double robotRotationOffset;
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

    private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);//TODO: Uncomment and reimplement when AHRS is updated for WPI 2023
    PIDController pid = new PIDController(Constants.DriveConstants.turnPID_P, Constants.DriveConstants.turnPID_I, Constants.DriveConstants.turnPID_D);
    
    
    //Shuffle Board//
    private GenericEntry maxSwerveShuffle(String name) {
        return Shuffleboard.getTab("Drive").add(name, SwerveDriveMathConstants.maxMotorUse_default)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1)).getEntry();
    }
    private GenericEntry motorSwerveShuffle(String name) {
        return Shuffleboard.getTab("Drive").add(name, "<0, 0>")
        .withWidget(BuiltInWidgets.kTextView).getEntry();
    }
    private GenericEntry encoderSwerveShuffle(String name) {
        return Shuffleboard.getTab("Drive").add(name, "N/A°")
        .withWidget(BuiltInWidgets.kTextView).getEntry();
    }
    private GenericEntry maxSwerveMotorUse = maxSwerveShuffle("Max Swerve Motor Use");
    private GenericEntry maxSwerveRotationSpeed = maxSwerveShuffle("Max Swerve Rotation Speed");
    private GenericEntry maxSwerveSpeed = maxSwerveShuffle("Max Swerve Speed");
    private GenericEntry[] swerveWheels = {motorSwerveShuffle("Front Left Wheel"), motorSwerveShuffle("Front Right Wheel"), motorSwerveShuffle("Back Left Wheel"), motorSwerveShuffle("Back Right Wheel")};
    private GenericEntry[] encoders = {encoderSwerveShuffle("Front Left Encoder"), encoderSwerveShuffle("Front Right Encoder"), encoderSwerveShuffle("Back Left Encoder"), encoderSwerveShuffle("Back Right Encoder")};
    /////////////////


    public SwerveDrive() {
    }
    public void Start() {
        pid.enableContinuousInput(-Math.PI, Math.PI);
        pid.setSetpoint(0);
        configureSparkBrake(frontLeftDriveMotor);
        configureSparkBrake(frontRightDriveMotor);
        configureSparkBrake(backLeftDriveMotor);
        configureSparkBrake(backRightDriveMotor);
    }
    

  private void configureSparkCoast(CANSparkMax sparkMax) {
    sparkMax.restoreFactoryDefaults();
    sparkMax.setOpenLoopRampRate(Constants.DriveConstants.kDTRampRate);
    sparkMax.setSmartCurrentLimit(Constants.DriveConstants.kCurrentLimit);
    sparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  private void configureSparkBrake(CANSparkMax sparkMax) {
    sparkMax.restoreFactoryDefaults();
    sparkMax.setOpenLoopRampRate(Constants.DriveConstants.kDTRampRate);
    sparkMax.setSmartCurrentLimit(Constants.DriveConstants.kCurrentLimit);
    sparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

    double StandardizeRAD(double radians) {
        return (radians + 8 * Math.PI) % (2 * Math.PI);
    }
    double GetEncoderAngle(CANCoder encoder, int ind) {
        return StandardizeRAD((encoder.getAbsolutePosition() - Constants.DriveConstants.turnAngleOffsets[ind]) * Math.PI / 180 * (Constants.DriveConstants.turnEncoderInversed[ind] ? 1 : -1));
    }
    Vector2 TurnPID(CANSparkMax motor, double currentAngle, double targetAngle) {
        double angleDif = Math.PI / 2 - (currentAngle - targetAngle - Math.PI/2 + Math.PI*5) % Math.PI;
        motor.set(pid.calculate(angleDif));
        return new Vector2(angleDif, Math.signum((currentAngle - targetAngle - Math.PI/2 + Math.PI*6) % (2*Math.PI) - Math.PI));
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
                robotRotation = -(m_navx.getFusedHeading()) * Math.PI / 180 - robotRotationOffset;
            } else {
                robotRotation = m_navx.getYaw() * Math.PI / 180 - robotRotationOffset;
            }
        }
        Vector2[] wheelSpeeds = SwerveDriveMath.WheelSpeeds(Math.min(joystickMag, 1), joystickAngle, joystickRotation, robotRotation);

        double frontLeftAngle = 2 * Math.PI - GetEncoderAngle(frontLeftEncoder, 0);
        double frontRightAngle = 2 * Math.PI - GetEncoderAngle(frontRightEncoder, 1);
        double backLeftAngle = 2 * Math.PI - GetEncoderAngle(backLeftEncoder, 2);
        double backRightAngle = 2 * Math.PI - GetEncoderAngle(backRightEncoder, 3);

        Vector2[] angleDifs = {new Vector2(), new Vector2(), new Vector2(), new Vector2()};
        if(joystickMag == 0 && joystickRotation == 0) {
            frontLeftTurnMotor.set(0);
            frontRightTurnMotor.set(0);
            backLeftTurnMotor.set(0);
            backRightTurnMotor.set(0);
        } else {
            angleDifs[0] = TurnPID(frontLeftTurnMotor, frontLeftAngle, wheelSpeeds[0].getAngle());
            angleDifs[1] = TurnPID(frontRightTurnMotor, frontRightAngle, wheelSpeeds[1].getAngle());
            angleDifs[2] = TurnPID(backLeftTurnMotor, backLeftAngle, wheelSpeeds[2].getAngle());
            angleDifs[3] = TurnPID(backRightTurnMotor, backRightAngle, wheelSpeeds[3].getAngle());
        }
        frontLeftDriveMotor.set(wheelSpeeds[0].getMagnitude() * angleDifs[0].y);
        frontRightDriveMotor.set(wheelSpeeds[1].getMagnitude() * angleDifs[1].y);
        backLeftDriveMotor.set(wheelSpeeds[2].getMagnitude() * angleDifs[2].y);
        backRightDriveMotor.set(wheelSpeeds[3].getMagnitude() * angleDifs[3].y);
        
        //Shuffle Board//
        for(int i = 0; i < wheelSpeeds.length; i++) {
            swerveWheels[i].setString(wheelSpeeds[i].toStringPolar(3));
        }
        DecimalFormat format = new DecimalFormat("#.###");
        encoders[0].setString(format.format(frontLeftAngle * 180 / Math.PI) + "° | Δ" + format.format(angleDifs[0].x * 180 / Math.PI) + "°");
        encoders[1].setString(format.format(frontRightAngle * 180 / Math.PI) + "° | Δ" + format.format(angleDifs[1].x * 180 / Math.PI) + "°");
        encoders[2].setString(format.format(backLeftAngle * 180 / Math.PI)+ "° | Δ" + format.format(angleDifs[2].x * 180 / Math.PI) + "°");
        encoders[3].setString(format.format(backRightAngle * 180 / Math.PI) + "° | Δ" + format.format(angleDifs[3].x * 180 / Math.PI) + "°");
        /////////////////

        return wheelSpeeds;
    }
    public void ResetRotation() {
        if(fieldOriented) {
            if (m_navx.isMagnetometerCalibrated()) {
                robotRotationOffset = -(m_navx.getFusedHeading()) * Math.PI / 180;
            } else {
                robotRotationOffset = m_navx.getYaw() * Math.PI / 180;
            }
        }
    }
}
