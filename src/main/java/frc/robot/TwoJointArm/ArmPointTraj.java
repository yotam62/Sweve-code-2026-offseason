package frc.robot.TwoJointArm;

public class ArmPointTraj {
    public final double shoulderAngle;
    public final double elbowAngle;
    public final double shoulderAngularVelocity;
    public final double elbowAngularVelocity;

    public ArmPointTraj(double shoulderAngle, double elbowAngle, double shoulderAngularVelocity, double elbowAngularVelocity) {
        this.shoulderAngle = shoulderAngle;
        this.elbowAngle = elbowAngle;
        this.shoulderAngularVelocity = shoulderAngularVelocity;
        this.elbowAngularVelocity = elbowAngularVelocity;
    }
    
}
