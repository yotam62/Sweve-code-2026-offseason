package frc.robot.TwoJointArm;

import frc.robot.Constants.ArmConstants;

public class ArmSubsystem {

    public record jointAngles( double shoulderAngle, double elbowAngle) {}

    public ArmSubsystem(ArmConstants constants) {


    }

    public jointAngles getJointAngles(ArmPose pose) {
        double x = pose.x;
        double z = pose.z;
        double shoulderLength = ArmConstants.shoulderLength;
        double elbowLength = ArmConstants.elbowLength;
        double elbowAngle;
        double shoulderAngle;
        double cosineElbowAngle;

        cosineElbowAngle = ((x*x + z*z -shoulderLength*shoulderLength - elbowLength*elbowLength)/(2*elbowLength*shoulderLength));
        Math.max(-1, Math.min(1, cosineElbowAngle));
        elbowAngle = -Math.acos(cosineElbowAngle);
        shoulderAngle = Math.atan2(z, x) + Math.atan2(elbowLength*Math.sin(elbowAngle), shoulderLength + elbowLength*Math.cos(elbowAngle));

        return new jointAngles(shoulderAngle, cosineElbowAngle);

    }
}
