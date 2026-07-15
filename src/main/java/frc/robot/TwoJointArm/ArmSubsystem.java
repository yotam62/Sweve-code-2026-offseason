package frc.robot.TwoJointArm;

import frc.robot.Constants.ArmConstants;

public class ArmSubsystem {

    public record jointXZToAngles(double shoulderAngle, double elbowAngle) {}
    public record jointAnglesToXZ(double x, double z) {}


    public ArmSubsystem(ArmConstants constants) {


    }

    public jointAnglesToXZ getXZ(double shoulderAngle, double elbowAngle){
        double shoulderLength = ArmConstants.shoulderLength;
        double elbowLength = ArmConstants.elbowLength;
        Double XValue;
        Double ZValue;

        XValue = Math.cos(shoulderAngle) + shoulderLength*Math.cos(shoulderAngle)+ elbowLength*Math.cos(elbowAngle+shoulderAngle);
        ZValue = Math.sin(shoulderAngle) + shoulderLength*Math.sin(shoulderAngle)+ elbowLength*Math.sin(elbowAngle+shoulderAngle);

        return new jointAnglesToXZ(XValue, ZValue);
    }

    public jointXZToAngles getJointAngles(ArmPose pose) {
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

        return new jointXZToAngles(shoulderAngle, cosineElbowAngle);

    }
}
