package frc.robot.TwoJointArm;

import frc.robot.Constants.ArmConstants;

public class ArmSubsystem {

    public record jointXZ(double shoulderAngle, double elbowAngle) {}
    public record jointAngles(double x, double z) {}


    public ArmSubsystem() {


    }


    public void setShoulderMotorVelocity(double Velocity) {
        
    }
}
