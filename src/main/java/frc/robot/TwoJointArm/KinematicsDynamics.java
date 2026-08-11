package frc.robot.TwoJointArm;

import frc.robot.Constants.ArmConstants;


public class KinematicsDynamics {

    public record jointXZ(double shoulderAngle, double elbowAngle) {}
    public record jointAngles(double x, double z) {}
    public record JointAccelerations(double shoulderAcceleration, double elbowAcceleration) {}

    public record JointVoltages(double shoulderVoltage, double elbowVoltage) {}

        public jointAngles getXZ(double shoulderAngle, double elbowAngle){
        double shoulderLength = ArmConstants.shoulderLength;
        double elbowLength = ArmConstants.elbowLength;

        double XValue = shoulderLength*Math.cos(shoulderAngle) + elbowLength*Math.cos(shoulderAngle+elbowAngle);
        double ZValue = shoulderLength*Math.sin(shoulderAngle) + elbowLength*Math.sin(shoulderAngle+elbowAngle);

        return new jointAngles(XValue, ZValue);
    }

    public static jointXZ getJointAngles(ArmPose pose) {
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

    public static JointVoltages getNeededVoltage(ArmPointTraj DesiredState){
        //Shoulder mass matrix
        double elbowAngle = DesiredState.elbowAngle();
        double shoulderAngle = DesiredState.shoulderAngle();
        double shoulderVelocity = DesiredState.shoulderAngularVelocity();
        double elbowVelocity = DesiredState.elbowAngularVelocity();
        double shoulderAcceleration = DesiredState.shoulderAcceleration();
        double elbowAcceleration = DesiredState.elbowAcceleration();
        double shoulderMOI = ArmConstants.shoulderMOI;
        double elbowMOI = ArmConstants.elbowMOI;
        double shoulderMass = ArmConstants.shoulderMass;
        double elbowMass = ArmConstants.elbowMass;
        double shoulderLength = ArmConstants.shoulderLength;
        double shoulderCGistance = ArmConstants.shoulderCGistance;
        double elbowCGdistance = ArmConstants.elbowCGistance;


        double shoulderGravityTorque = shoulderMass * 9.81 * shoulderCGistance * Math.cos(shoulderAngle) + elbowMass * 9.81 * (shoulderLength * Math.cos(shoulderAngle) + elbowCGdistance * Math.cos(shoulderAngle + elbowAngle));
        double elbowGravityTorque = elbowMass * 9.81 * elbowCGdistance * Math.cos(shoulderAngle + elbowAngle);

        double VelocityCouplingFactor = -(elbowMass * shoulderLength * elbowCGdistance * Math.sin(elbowAngle));
        double elbowVelocityTourque = -VelocityCouplingFactor * shoulderVelocity * shoulderVelocity;
        double shoulderVelocityTorque = VelocityCouplingFactor * elbowVelocity * elbowVelocity + 2 *VelocityCouplingFactor * shoulderVelocity * elbowVelocity;

        double M1 = shoulderMOI + elbowMOI + elbowMass * shoulderLength * shoulderLength 
            + 2 * elbowMass * shoulderLength * elbowCGdistance * Math.cos(elbowAngle);
        double M2 = elbowMOI + elbowMass * shoulderLength * elbowCGdistance * Math.cos(elbowAngle);
        double M3 = M2;
        double M4 = elbowMOI;


        double shoulderTorque = M1 * shoulderAcceleration + M2 * elbowAcceleration + shoulderGravityTorque + shoulderVelocityTorque;
        double elbowTorque    = M3 * shoulderAcceleration + M4 * elbowAcceleration + elbowGravityTorque + elbowVelocityTourque;

        return new JointVoltages(ArmConstants.shoulderMotor.getVoltage(shoulderTorque, DesiredState.shoulderAngularVelocity()), ArmConstants.elbowMotor.getVoltage(elbowTorque, DesiredState.elbowAngularVelocity()));


    }

    public static JointAccelerations getAccelerations(
        double appliedShoulderTorque, double appliedElbowTorque,
        double shoulderAngle, double elbowAngle,
        double shoulderVelocity, double elbowVelocity) {

    // --- Same building blocks as getNeededVoltage, state-only, no accel needed ---
    double shoulderMOI = ArmConstants.shoulderMOI;
    double elbowMOI = ArmConstants.elbowMOI;
    double shoulderMass = ArmConstants.shoulderMass;
    double elbowMass = ArmConstants.elbowMass;
    double shoulderLength = ArmConstants.shoulderLength;
    double shoulderCGistance = ArmConstants.shoulderCGistance;
    double elbowCGdistance = ArmConstants.elbowCGistance;

    double shoulderGravityTorque = shoulderMass * 9.81 * shoulderCGistance * Math.cos(shoulderAngle)
        + elbowMass * 9.81 * (shoulderLength * Math.cos(shoulderAngle) + elbowCGdistance * Math.cos(shoulderAngle + elbowAngle));
    double elbowGravityTorque = elbowMass * 9.81 * elbowCGdistance * Math.cos(shoulderAngle + elbowAngle);

    double VelocityCouplingFactor = -(elbowMass * shoulderLength * elbowCGdistance * Math.sin(elbowAngle));
    double elbowVelocityTourque = -VelocityCouplingFactor * shoulderVelocity * shoulderVelocity;
    double shoulderVelocityTorque = VelocityCouplingFactor * elbowVelocity * elbowVelocity + 2 * VelocityCouplingFactor * shoulderVelocity * elbowVelocity;

    double M1 = shoulderMOI + elbowMOI + elbowMass * shoulderLength * shoulderLength
        + 2 * elbowMass * shoulderLength * elbowCGdistance * Math.cos(elbowAngle);
    double M2 = elbowMOI + elbowMass * shoulderLength * elbowCGdistance * Math.cos(elbowAngle);
    double M3 = M2;
    double M4 = elbowMOI;

    double b1 = appliedShoulderTorque - shoulderGravityTorque - shoulderVelocityTorque;
    double b2 = appliedElbowTorque - elbowGravityTorque - elbowVelocityTourque;

    double determinant = M1 * M4 - M2 * M3;

    double shoulderAcceleration = (M4 * b1 - M2 * b2) / determinant;
    double elbowAcceleration    = (M1 * b2 - M3 * b1) / determinant;

    return new JointAccelerations(shoulderAcceleration, elbowAcceleration);
}


}