package frc.robot;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

    public SwerveDrive() {
        //bug test this
        //frontLeftTurnMotor.getPIDController().setFeedbackDevice​(frontLeftEncoder);
        //frontRightTurnMotor.getPIDController().setFeedbackDevice​(frontRightEncoder);
        //backLeftTurnMotor.getPIDController().setFeedbackDevice​(backLeftEncoder);
        //backRightTurnMotor.getPIDController().setFeedbackDevice​(backRightEncoder);
        configurePID(frontLeftTurnMotor);
        configurePID(frontRightTurnMotor);
        configurePID(backLeftTurnMotor);
        configurePID(backRightTurnMotor);
    }
    void configurePID(CANSparkMax motor) {
        //configure PID in here
    }
    public Vector2[] SetWheelSpeeds(double joystickMag, double joystickAngle, double joystickRotation) {
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
        
        //frontLeftTurnMotor.getPIDController().setReference(wheelSpeeds[0].getAngle() - robotRotation + Constants.DriveConstants.turnAngleOffsets[0], ControlType.kPosition);
        //frontRightTurnMotor.getPIDController().setReference(wheelSpeeds[1].getAngle() - robotRotation + Constants.DriveConstants.turnAngleOffsets[1], ControlType.kPosition);
        //backLeftTurnMotor.getPIDController().setReference(wheelSpeeds[2].getAngle() - robotRotation + Constants.DriveConstants.turnAngleOffsets[2], ControlType.kPosition);
        //backRightTurnMotor.getPIDController().setReference(wheelSpeeds[3].getAngle() - robotRotation + Constants.DriveConstants.turnAngleOffsets[3], ControlType.kPosition);
        frontLeftTurnMotor.set(pid.calculate(frontLeftEncoder.getPosition() * Math.PI / 180, wheelSpeeds[0].getAngle()));
        frontRightTurnMotor.set(pid.calculate(frontRightEncoder.getPosition() * Math.PI / 180, wheelSpeeds[1].getAngle()));
        backLeftTurnMotor.set(pid.calculate(backLeftEncoder.getPosition() * Math.PI / 180, wheelSpeeds[2].getAngle()));
        backRightTurnMotor.set(pid.calculate(backRightEncoder.getPosition() * Math.PI / 180, wheelSpeeds[3].getAngle()));
        return wheelSpeeds;
    }
}
