package frc.robot;

public class SwerveDriveMath {
    //helper functions
    private static double rotX(double x, double y, double rot) {
      double angle = Math.atan(y/x);
      if(x < 0) {
        angle += Math.PI;
      }
      return Math.cos(angle - rot) * Math.sqrt(x*x + y*y);  
    }
    
    private static double rotY(double x, double y, double rot) {
      double angle = Math.atan(y/x);
      if(x < 0) {
        angle += Math.PI;
      }
      return Math.sin(angle - rot) * Math.sqrt(x*x + y*y);  
    }
    //
  
    //math
    public static double maxNonLimitSpeed(double pRADS)//gets the maximum speed that's available at every movement direction (decreases as the rotation speed increases)
     {
      return Math.min((2*SwerveDriveMathConstants.maxMotorUse - Math.abs(SwerveDriveMathConstants.hypotenuse * pRADS)) / (2 * SwerveDriveMathConstants.maxMoveSpeed), 1);
    }
    private static double velocityMagnitude(double jMAG, double jRAD, double pRADS) {
      int mod = ((int)Math.ceil(2 * jRAD / Math.PI) % 2);
      double frac = SwerveDriveMathConstants.robotWidth/SwerveDriveMathConstants.robotHeight;
      if(mod == 0) {
        frac = 1/frac;
      }
      double a = jRAD - Math.atan(frac);
      double cos = Math.cos(a) * SwerveDriveMathConstants.hypotenuse;
      double sin = Math.sin(a) * SwerveDriveMathConstants.hypotenuse;
      if(mod == 0) {
        double temp = cos;
        cos = sin;
        sin = temp;
      }
      return Math.min((-Math.abs(cos * pRADS) + Math.sqrt(-sin*sin*pRADS*pRADS + 4 * SwerveDriveMathConstants.maxMotorUse*SwerveDriveMathConstants.maxMotorUse)) / 2, jMAG * SwerveDriveMathConstants.maxMoveSpeed);
    }
    //  
  public static Vector2[] WheelSpeeds(
    double joystickMag,//(0-1) //magnitude of move joystick
    double joystickAngle,//(0-2pi) //angle of move joystick
    double joystickRotation,//(-1) to 1 //value of turn joystick
    double robotRotation//0-2pi //robot's rotation
    ) {
    //math
    ////
    double joystickTrue = joystickAngle + robotRotation;
    double pRADS = joystickRotation * SwerveDriveMathConstants.maxRotationSpeed;
    //
    double robotSpeed = velocityMagnitude(joystickMag,  joystickTrue, pRADS);
    double robotSpeedX = Math.cos(joystickTrue) * robotSpeed;
    double robotSpeedY = Math.sin(joystickTrue) * robotSpeed;
    ////
    double w = pRADS*SwerveDriveMathConstants.robotWidth / 2;
    double h = pRADS*SwerveDriveMathConstants.robotHeight / 2;
    //
    Vector2 frontLeft = new Vector2(robotSpeedX + h, robotSpeedY + w);
    Vector2 frontRight = new Vector2(robotSpeedX + h, robotSpeedY - w);
    Vector2 backLeft = new Vector2(robotSpeedX - h, robotSpeedY + w);
    Vector2 backRight = new Vector2(robotSpeedX - h, robotSpeedY - w);
    //
    return new Vector2[] {frontLeft, frontRight, backLeft, backRight};
  }
}
  