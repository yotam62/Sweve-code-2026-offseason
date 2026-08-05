package frc.robot.TwoJointArm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TwoJointArm.ArmIO.ArmIOinputs;


public class ArmSubsystem extends SubsystemBase {

    public record jointXZ(double shoulderAngle, double elbowAngle) {}
    public record jointAngles(double x, double z) {}

    private final ArmIO io;
    private final ArmIOinputs inputs = new ArmIOinputs();

    public ArmSubsystem(ArmIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setVolatge(double shoulderVolts, double elbowVolts) {
        io.setVoltage(shoulderVolts, elbowVolts);
    }

    public ArmPointTraj getcurrentState(){
        return new ArmPointTraj(
        inputs.shoulderAngle, inputs.elbowAngle,
        inputs.shoulderVelocity, inputs.elbowVelocity,
        0,0
        );
    }
}
