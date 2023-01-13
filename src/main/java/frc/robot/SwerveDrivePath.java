package frc.robot;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import org.apache.commons.math3.complex.Complex;

import edu.wpi.first.math.controller.PIDController;

public class SwerveDrivePath {
    Vector2 startPos;
    Vector2 startVelocity;
    Vector2 endPos;
    Vector2 endVelocity;
  
    double mx1;
    double mx2;
    double mx3;
    double my1;
    double my2;
    double my3;
    double mult;
    public SwerveDrivePath(Vector2 startPos, Vector2 startVelocity, Vector2 endPos, Vector2 endVelocity) {
        this.startPos = startPos;
        this.startVelocity = startVelocity;
        this.endPos = endPos;
        this.endVelocity = endVelocity;

        //Math stuff//
        //https://www.desmos.com/calculator/jpy4q7hgfy
        Vector2 posDif = new Vector2(endPos.x - startPos.x, endPos.y - startPos.y);
        mult = Math.sqrt(posDif.x*posDif.x + posDif.y*posDif.y);
        mx1 = mult * startVelocity.x;
        my1 = mult * startVelocity.y;

        mx2 = 3 * posDif.x - 2 * mx1 - endVelocity.x * mult;
        my2 = 3 * posDif.y - 2 * my1 - endVelocity.y * mult;

        mx3 = posDif.x - mx2 - mx1;
        my3 = posDif.y - my2 - my1;
        //////////////
    }
    public Vector2 GetRobotSpeed(Vector2 position, PIDController pid) {
        return new Vector2();
    }
    public Vector2 GetClosestTime(Vector2 position)// returns (time, distance)
     {
        //More Math stuff//
        //https://www.desmos.com/calculator/jpy4q7hgfy
        Vector2 posDif = new Vector2(startPos.x - position.x, startPos.y - position.y);
        
        double v6 = mx3 * mx3 + my3 * my3;
        double v5 = 2 * (my2 * my3 + mx2 *mx3);
        double v4 = 2 * my1 * my3 + my2 * my2 + 2 * mx1 * mx3 + mx2 * mx2;
        double v3 = 2 * (my3 * posDif.y + my1 * my2 + mx3 * posDif.x + mx1 * mx2);
        double v2 = 2 * my2 * posDif.y + my1 * my1 + 2 * mx2 * posDif.x + mx1 * mx1;
        double v1 = 2 * (my1 * posDif.y + mx1 * posDif.x);
        double v0 = posDif.y * posDif.y + posDif.x * posDif.x;
        LaguerreSolver solver = new LaguerreSolver();
        Complex[] solutions = solver.solveAllComplex(new double[] {v1, 2*v2, 3*v3, 4*v4, 5*v5, 6*v6}, 0);

        double closestTime = 0;
        double closestDistSquared = v0;
        double testClosestDistSquared = v6 + v5 + v4 + v3 + v2 + v1 + v0;
        if(testClosestDistSquared < closestDistSquared) {
            closestDistSquared = testClosestDistSquared;
            closestTime = 1;
        }
        for(int i = 0; i < solutions.length; i++) {
            Complex value = solutions[i];
            double real = value.getReal();
            if(value.getImaginary() == 0 && real > 0 && real < 1) {
                double dist = v0;
                double temp = real;
                dist += v1 * temp;
                temp*=real;
                dist += v2 * temp;
                temp*=real;
                dist += v3 * temp;
                temp*=real;
                dist += v4 * temp;
                temp*=real;
                dist += v5 * temp;
                temp*=real;
                dist += v6 * temp;
                if(dist < closestDistSquared) {
                    closestDistSquared = dist;
                    closestTime = real;
                }
            }
        }
        return new Vector2(closestTime, Math.sqrt(closestDistSquared));
      ///////////////////
    }

    public Vector2 Velocity(double time) // magnitude can go over 1. Cap magnitude of resulting vector if you plan to use this to set motor speeds directly
    {
      double x = mx1; 
      double y = my1;
      x += 2 * mx2 * time;
      y += 2 * my2 * time;
      time *= time;
      x += 3 * mx3 * time;
      y += 3 * my3 * time;
      return new Vector2(x/mult, y/mult);
    }
  
    public Vector2 Position(double time) {
      double x = startPos.x; 
      double y = startPos.y;
      double temp = time;
      
      x += mx1 * temp;
      y += my1 * temp;
      temp *= time;
      x += mx2 * temp;
      y += my2 * temp;
      temp *= time;
      x += mx3 * temp;
      y += my3 * temp;
      return new Vector2(x, y);
    }
}
