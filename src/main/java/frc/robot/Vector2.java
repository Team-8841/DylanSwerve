package frc.robot;

public class Vector2 {
    public double x;
    public double y;
    public Vector2(double x, double y) {
      this.x = x;
      this.y = y;
    }
    public double getAngle() {
      if(x == 0 && y == 0) {
          return 0;
      }
      if(x < 0) {
        return Math.atan(y/x) + Math.PI;
      } else {
        return (Math.atan(y/x) + 2 * Math.PI) % (2 * Math.PI);
      }
    }
    public double getMagnitude() {
      return Math.sqrt(x*x + y*y);
    }
    public String toString() {
      return "<" + x + ", " + y + ">";
    }
    public String toString(int decimals) {
      double pow = Math.pow(10, decimals);
      return "<" + Math.round(x*pow) / pow + ", " + Math.round(y*pow) / pow + ">";
    }
    public String toStringPolar(int decimals) {
      double pow = Math.pow(10, decimals);
      return "<" + Math.round(getMagnitude()*pow) / pow + ", " + Math.round(getAngle() * 180 / Math.PI *pow) / pow + "°>";
    }
  }