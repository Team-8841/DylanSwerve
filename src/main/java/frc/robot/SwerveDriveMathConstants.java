package frc.robot;

public class SwerveDriveMathConstants {
    public static final double robotWidth = 0.54; //distance between front motors
    public static final double robotHeight = 0.54; //distance between front and back motors
    ////
    public static final double maxMotorUse_default = 1;//0-1 //maximum usage of all motors
    public static final double maxRotationSlider_default = 0.5;//0-1 //max rotation motor usage
    public static final double maxMoveSpeed_default = 0.9;//0-1 //max move motor usage
    //
    public static double maxMotorUse = maxMotorUse_default;
    public static double maxRotationSlider = maxRotationSlider_default;
    public static double maxMoveSpeed = maxMoveSpeed_default;
    ////
  
  
    //don't worry about it
    public static final double hypotenuse = Math.sqrt(robotHeight*robotHeight + robotWidth*robotWidth);
    
    public static double maxRotationSpeed = maxRotationSlider * (2*maxMotorUse)/hypotenuse;
    public static double maxNonLimitRotation = 2 * (maxMotorUse - maxMoveSpeed)/(maxRotationSpeed * hypotenuse);

    public static void updateMax() {
        maxRotationSpeed = maxRotationSlider * (2*maxMotorUse)/hypotenuse;
        maxNonLimitRotation = 2 * (maxMotorUse - maxMoveSpeed)/(maxRotationSpeed * hypotenuse);
    }
    //
  }