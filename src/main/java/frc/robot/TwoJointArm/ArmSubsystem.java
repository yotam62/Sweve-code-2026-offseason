package frc.robot.TwoJointArm;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem {

    public record jointXZ(double shoulderAngle, double elbowAngle) {}
    public record jointAngles(double x, double z) {}

    DCMotor shoulderMotor = new DCMotor(12, 7.09, 366, 2, 6000, 1);
    DCMotor elbowMotor = new DCMotor(12, 7.09, 366, 2, 6000, 1);

    public ArmSubsystem() {


    }

    public jointAngles getXZ(double shoulderAngle, double elbowAngle){
        double shoulderLength = ArmConstants.shoulderLength;
        double elbowLength = ArmConstants.elbowLength;
        Double XValue;
        Double ZValue;

        XValue = shoulderLength*Math.cos(shoulderAngle) + elbowLength*Math.cos(shoulderAngle+elbowAngle);
        ZValue = shoulderLength*Math.sin(shoulderAngle) + elbowLength*Math.sin(shoulderAngle+elbowAngle);

        return new jointAngles(XValue, ZValue);
    }

    public jointXZ getJointAngles(ArmPose pose) {
        double x = pose.x;
        double z = pose.z;
        double shoulderLength = ArmConstants.shoulderLength;
        double elbowLength = ArmConstants.elbowLength;
        double elbowAngle;
        double shoulderAngle;
        double cosineElbowAngle;

        cosineElbowAngle = ((x*x + z*z -shoulderLength*shoulderLength - elbowLength*elbowLength)/(2*elbowLength*shoulderLength));
        cosineElbowAngle = Math.max(-1, Math.min(1, cosineElbowAngle));
        elbowAngle = -Math.acos(cosineElbowAngle);
        shoulderAngle = Math.atan2(z, x) - Math.atan2(elbowLength*Math.sin(elbowAngle), shoulderLength + elbowLength*Math.cos(elbowAngle));

        return new jointXZ(shoulderAngle, elbowAngle);

    }

    public void setShoulderMotorVelocity(double Velocity) {
        
    }
}
