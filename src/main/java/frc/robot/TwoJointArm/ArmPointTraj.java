package frc.robot.TwoJointArm;

public class ArmPointTraj {
    public final double shoulderAngle;
    public final double elbowAngle;
    public final double shoulderAngularVelocity;
    public final double elbowAngularVelocity;
    public final double shoulderAcceleration;
    public final double elbowAcceleration;

    public ArmPointTraj(
    double shoulderAngle, double elbowAngle, 
    double shoulderAngularVelocity, double elbowAngularVelocity,
    double shoulderAcceleration, double elbowAcceleration) {
        this.shoulderAngle = shoulderAngle;
        this.elbowAngle = elbowAngle;
        this.shoulderAngularVelocity = shoulderAngularVelocity;
        this.elbowAngularVelocity = elbowAngularVelocity;
        this.shoulderAcceleration = shoulderAcceleration;
        this.elbowAcceleration = elbowAcceleration;
    }
    
}
