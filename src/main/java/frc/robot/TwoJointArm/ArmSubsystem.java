package frc.robot.TwoJointArm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.TwoJointArm.ArmCommands.ArmSim;
import frc.robot.TwoJointArm.ArmCommands.ArmSim.SimState;
import frc.robot.TwoJointArm.ArmIO.ArmIOinputs;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;


public class ArmSubsystem extends SubsystemBase {

    private final Mechanism2d armMechanism = new Mechanism2d(3, 5);
    private final MechanismRoot2d root = armMechanism.getRoot("ShoulderJoint", 1.5, 0.5);
    private final MechanismLigament2d shoulderLigament =
        root.append(new MechanismLigament2d("shoulder", ArmConstants.shoulderLength, 0, 6, new Color8Bit(Color.kRed)));
    private final MechanismLigament2d elbowLigament = shoulderLigament.append(new MechanismLigament2d("elbow", ArmConstants.elbowLength, 0, 6 ,new Color8Bit(Color.kBlue) ));

    public record jointXZ(double shoulderAngle, double elbowAngle) {}
    public record jointAngles(double x, double z) {}

    ArmIO io;
    ArmIOinputsAutoLogged inputs = new ArmIOinputsAutoLogged();


    public ArmSubsystem(ArmIO io) {
        this.io = io;

        SmartDashboard.putData("ArmMechanism", armMechanism);


    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ArmInputs", inputs);

        shoulderLigament.setAngle(Units.radiansToDegrees(inputs.shoulderAngle));
        elbowLigament.setAngle(Units.radiansToDegrees(inputs.elbowAngle));
    }

    public void setVoltage(double shoulderVolts, double elbowVolts) {
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
