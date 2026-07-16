package frc.robot.TwoJointArm;

import java.util.ArrayList;
import java.util.List;
import frc.robot.TwoJointArm.ArmPointTraj;

import frc.robot.Constants;

public class TrapezoidalTrajGenerator implements TrajGeneratorInterface {
    
    public double shoulderMaxVelocity = Constants.ArmConstants.shoulderMaxVelocity;
    public double elbowMaxVelocity = Constants.ArmConstants.elbowMaxVelocity;
    public double shoulderMaxAcceleration = Constants.ArmConstants.shoulderMaxAcceleration;
    public double elbowMaxAcceleration = Constants.ArmConstants.elbowMaxAcceleration;
    public double shoulderMaxDeceleration = Constants.ArmConstants.shoulderMaxDeceleration;
    public double elbowMaxDeceleration = Constants.ArmConstants.elbowMaxDeceleration;

    public double shoulderAccelTime = shoulderMaxVelocity / shoulderMaxAcceleration;
    public double elbowAccelTime = elbowMaxVelocity / elbowMaxAcceleration;
    public double shoulderDecelTime = shoulderMaxVelocity / shoulderMaxDeceleration;
    public double elbowDecelTime = elbowMaxVelocity / elbowMaxDeceleration;


    public double shoulderAccelDistance = (shoulderMaxVelocity * shoulderMaxVelocity)  / (shoulderMaxAcceleration *2);
    public double elbowAccelDistance = (elbowMaxVelocity * elbowMaxVelocity)  / (elbowMaxAcceleration *2);
    public double shoulderDecelDistance = (shoulderMaxVelocity * shoulderMaxVelocity)  / (shoulderMaxDeceleration *2);
    public double elbowDecelDistance = (elbowMaxVelocity * elbowMaxVelocity)  / (elbowMaxDeceleration *2);




    




    @Override
    public ArmTraj createTraj(ArmPointTraj start, ArmPointTraj goal, List<Obstacle> obstacle) {


                
         double CshoulderMaxVelocity = shoulderMaxVelocity;//
         double CelbowMaxVelocity = elbowMaxVelocity;//
         double CshoulderMaxAcceleration = shoulderMaxAcceleration;//
         double CelbowMaxAcceleration = elbowMaxAcceleration;//
         double CshoulderMaxDeceleration = shoulderMaxDeceleration;//
         double CelbowMaxDeceleration = elbowMaxDeceleration; //

         double CshoulderAccelTime = CshoulderMaxVelocity / CshoulderMaxAcceleration;
         double CelbowAccelTime = CelbowMaxVelocity / CelbowMaxAcceleration;
         double CshoulderDecelTime = CshoulderMaxVelocity / CshoulderMaxDeceleration;
         double CelbowDecelTime = CelbowMaxVelocity / CelbowMaxDeceleration;

         double CshoulderAccelDistance = (CshoulderMaxVelocity * CshoulderMaxVelocity)  / (CshoulderMaxAcceleration *2);
         double CelbowAccelDistance = (CelbowMaxVelocity * CelbowMaxVelocity)  / (CelbowMaxAcceleration *2);
         double CshoulderDecelDistance = (CshoulderMaxVelocity * CshoulderMaxVelocity)  / (CshoulderMaxDeceleration *2);
         double CelbowDecelDistance = (CelbowMaxVelocity * CelbowMaxVelocity)  / (CelbowMaxDeceleration *2);

            List<ArmPointTraj> pointList = new ArrayList<>();
            List<Double> timeStamps = new ArrayList<>();
            double totalShoulderDistance = Math.abs(goal.shoulderAngle() - start.shoulderAngle());
            double totalElbowDistance = Math.abs(goal.elbowAngle() - start.elbowAngle());
            double shoulderCruiseDistance = totalShoulderDistance - CshoulderAccelDistance - CshoulderDecelDistance;
            double elbowCruiseDistance = totalElbowDistance - CelbowAccelDistance - CelbowDecelDistance;
            double shoulderCruiseTime = shoulderCruiseDistance / CshoulderMaxVelocity;
            double elbowCruiseTime = elbowCruiseDistance / CelbowMaxVelocity;

             double totalTime = Math.max(elbowAccelTime + elbowCruiseTime + elbowDecelTime, shoulderAccelTime + shoulderCruiseTime + shoulderDecelTime);
             double shoulderDir = Math.signum(goal.shoulderAngle() - start.shoulderAngle());
             double elbowDir = Math.signum(goal.elbowAngle() - start.elbowAngle());

            if (shoulderAccelDistance + shoulderDecelDistance > Math.abs(goal.shoulderAngle() - start.shoulderAngle())) {
                CshoulderMaxVelocity = Math.sqrt(2*(Math.abs(goal.shoulderAngle() - start.shoulderAngle()) * CshoulderMaxAcceleration * CshoulderMaxDeceleration) / (CshoulderMaxAcceleration + CshoulderMaxDeceleration));
                CshoulderAccelTime = CshoulderMaxVelocity / CshoulderMaxAcceleration;
                CshoulderDecelTime = CshoulderMaxVelocity / CshoulderMaxDeceleration;
                shoulderCruiseDistance = 0;
                shoulderCruiseTime = 0;

            }

            if (elbowAccelDistance + CelbowDecelDistance > Math.abs(goal.elbowAngle() - start.elbowAngle())) {
                CelbowMaxVelocity = Math.sqrt(2*(Math.abs(goal.elbowAngle() - start.elbowAngle()) * CelbowMaxAcceleration * CelbowMaxDeceleration) / (CelbowMaxAcceleration + CelbowMaxDeceleration));
                CelbowAccelTime = CelbowMaxVelocity / CelbowMaxAcceleration;
                CelbowDecelTime = CelbowMaxVelocity / CelbowMaxDeceleration;
                elbowCruiseDistance = 0;
                elbowCruiseTime = 0;
            }


            totalTime = Math.max(CelbowAccelTime + elbowCruiseTime + CelbowDecelTime, CshoulderAccelTime + shoulderCruiseTime + CshoulderDecelTime);
             
        for ( double i = 0; i < totalTime; i += Constants.ArmConstants.dt) {

            if (i < CelbowAccelTime) {
                if (i < CshoulderAccelTime) {
                    pointList.add(new ArmPointTraj(start.shoulderAngle() + shoulderDir * 0.5 * CshoulderMaxAcceleration * i * i, start.elbowAngle() + elbowDir * 0.5 * CelbowMaxAcceleration * i * i, CshoulderMaxAcceleration * i * shoulderDir, CelbowMaxAcceleration * i * elbowDir, CshoulderMaxAcceleration, CelbowMaxAcceleration));
                    timeStamps.add(i);
                }
                else if (i > CshoulderAccelTime + shoulderCruiseTime) {
                    pointList.add(new ArmPointTraj(goal.shoulderAngle() - shoulderDir * 0.5 * CshoulderMaxDeceleration * (totalTime - i) * (totalTime - i), start.elbowAngle() + elbowDir * 0.5 * CelbowMaxAcceleration * i * i, CshoulderMaxDeceleration * (totalTime - i) * shoulderDir, CelbowMaxAcceleration * i * elbowDir, -CshoulderMaxDeceleration, CelbowMaxAcceleration));
                    timeStamps.add(i);
                }

                else {
                    pointList.add(new ArmPointTraj(start.shoulderAngle() + shoulderDir * (0.5 * CshoulderMaxAcceleration * shoulderAccelTime * shoulderAccelTime + CshoulderMaxVelocity * (i - shoulderAccelTime)), start.elbowAngle() + elbowDir * 0.5 * CelbowMaxAcceleration * i * i, CshoulderMaxVelocity * shoulderDir, CelbowMaxAcceleration * i * elbowDir, 0, CelbowMaxAcceleration));
                    timeStamps.add(i);
                }


            }

            else if(i > CelbowAccelTime + elbowCruiseTime) {
                

                if (i > CshoulderAccelTime + shoulderCruiseTime) {
                                    if (i < CshoulderAccelTime) {
                    pointList.add(new ArmPointTraj(start.shoulderAngle() + shoulderDir * 0.5 * CshoulderMaxAcceleration * i * i, start.elbowAngle() + elbowDir * 0.5 * CelbowMaxAcceleration * i * i, CshoulderMaxAcceleration * i * shoulderDir, CelbowMaxAcceleration * i * elbowDir, CshoulderMaxAcceleration, CelbowMaxAcceleration));
                    timeStamps.add(i);
                }

                else {
                    pointList.add(new ArmPointTraj(start.shoulderAngle() + shoulderDir * (0.5 * CshoulderMaxAcceleration * shoulderAccelTime * shoulderAccelTime + CshoulderMaxVelocity * (i - shoulderAccelTime)), start.elbowAngle() + elbowDir * 0.5 * CelbowMaxAcceleration * i * i, CshoulderMaxVelocity * shoulderDir, CelbowMaxAcceleration * i * elbowDir, 0, CelbowMaxAcceleration));
                    timeStamps.add(i);  
                }

                }

            }

            else {
                if (i < CshoulderAccelTime) {
                    pointList.add(new ArmPointTraj(start.shoulderAngle() + shoulderDir * 0.5 * CshoulderMaxAcceleration * i * i, start.elbowAngle() + elbowDir * 0.5 * CelbowMaxAcceleration * i * i, CshoulderMaxAcceleration * i * shoulderDir, CelbowMaxAcceleration * i * elbowDir, CshoulderMaxAcceleration, CelbowMaxAcceleration));
                    timeStamps.add(i);
                }
                else if (i > CshoulderAccelTime + shoulderCruiseTime) {
                    pointList.add(new ArmPointTraj(goal.shoulderAngle() - shoulderDir * 0.5 * CshoulderMaxDeceleration * (totalTime - i) * (totalTime - i), start.elbowAngle() + elbowDir * 0.5 * CelbowMaxAcceleration * i * i, CshoulderMaxDeceleration * (totalTime - i) * shoulderDir, CelbowMaxAcceleration * i * elbowDir, -CshoulderMaxDeceleration, CelbowMaxAcceleration));
                    timeStamps.add(i);

                }

                else {
                    pointList.add(new ArmPointTraj(start.shoulderAngle() + shoulderDir * (0.5 * CshoulderMaxAcceleration * shoulderAccelTime * shoulderAccelTime + CshoulderMaxVelocity * (i - shoulderAccelTime)), start.elbowAngle() + elbowDir * (0.5 * CelbowMaxAcceleration * elbowAccelTime * elbowAccelTime + CelbowMaxVelocity * (i - elbowAccelTime)), CshoulderMaxVelocity * shoulderDir, CelbowMaxVelocity * elbowDir, 0, 0));
                    timeStamps.add(i);
                }

            }
            

        }

         return new ArmTraj(pointList, totalTime, timeStamps);
    
} 

}
//deal with timestamps -every time I add a point to the list I add it's timestamp as well