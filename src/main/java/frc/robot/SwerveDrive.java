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
    

    public SwerveDrive() {
        
    }
    public Vector2[] SetWheelSpeeds(double joystickMag, double joystickAngle, double joystickRotation) {
        double robotRotation = 0; //TODO: implement gyroscope later // may need to subtract instead of add
        Vector2[] wheelSpeeds = SwerveDriveMath.WheelSpeeds(Math.min(joystickMag, 1), joystickAngle, joystickRotation, robotRotation);
        frontLeftDriveMotor.set(wheelSpeeds[0].getMagnitude());
        frontRightDriveMotor.set(wheelSpeeds[1].getMagnitude());
        backLeftDriveMotor.set(wheelSpeeds[2].getMagnitude());
        backRightDriveMotor.set(wheelSpeeds[3].getMagnitude());
        
        frontLeftTurnMotor.getPIDController().setReference(wheelSpeeds[0].getAngle() + robotRotation + Constants.DriveConstants.turnAngleOffsets[0], ControlType.kPosition);
        frontRightTurnMotor.getPIDController().setReference(wheelSpeeds[1].getAngle() + robotRotation + Constants.DriveConstants.turnAngleOffsets[1], ControlType.kPosition);
        backLeftTurnMotor.getPIDController().setReference(wheelSpeeds[2].getAngle() + robotRotation + Constants.DriveConstants.turnAngleOffsets[2], ControlType.kPosition);
        backRightTurnMotor.getPIDController().setReference(wheelSpeeds[3].getAngle() + robotRotation + Constants.DriveConstants.turnAngleOffsets[3], ControlType.kPosition);

        return wheelSpeeds;
    }
}
