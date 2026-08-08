package frc.robot.TwoJointArm.ArmCommands;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.TwoJointArm.ArmIO;
import frc.robot.TwoJointArm.ArmPointTraj;
import frc.robot.TwoJointArm.KinematicsDynamics;
import frc.robot.TwoJointArm.KinematicsDynamics.JointAccelerations;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.NumericalIntegration;


/*ok so the plan is this: In the big-ass function I take the current state's positions and speeds of the joints, 
and the forward dynamics (meaning if I plug x voltage what will the accel and speed and be, which I use the dynamics class and changes
based on mass matrix and shi like that). anyway I plug it all into the rk4 method that baysically calculates the accel and speed 
in 4 points instead of 1 and gives us a more detailed view of how the robot will act, and than the function spits out the 4 numbers:
 the new joint positions and speeds in the next point */
public class ArmSim implements ArmIO {

    public record SimState(double shoulderAngle, double elbowAngle, double shoulderVelocity, double elbowVelocity) {}

    
    private SimState currentState = new SimState(Constants.ArmConstants.shoulderStartingAngle, Constants.ArmConstants.elbowStartingAngle, 0, 0); // or whatever your real start angles are
    private double appliedShoulderVoltage = 0.0;
    private double appliedElbowVoltage = 0.0;

    public SimState getNextState(SimState previousState) {

        double currentShoulderAngle = previousState.shoulderAngle();
        double currentElbowAngle = previousState.elbowAngle();
        double currentShoulderVelocity = previousState.shoulderVelocity();
        double currentElbowVelocity = previousState.elbowVelocity();

        
        Matrix<N4, N1> x = VecBuilder.fill(
            currentShoulderAngle, currentElbowAngle,
            currentShoulderVelocity, currentElbowVelocity);

        
        Matrix<N2, N1> u = VecBuilder.fill(appliedShoulderVoltage, appliedElbowVoltage);

        Matrix<N4, N1> nextState = NumericalIntegration.rk4(
            (Matrix<N4, N1> xIn, Matrix<N2, N1> uIn) -> {
                double shoulderAngle    = xIn.get(0, 0);
                double elbowAngle       = xIn.get(1, 0);
                double shoulderVelocity = xIn.get(2, 0);
                double elbowVelocity    = xIn.get(3, 0);

                double shoulderVoltage = uIn.get(0, 0);
                double elbowVoltage    = uIn.get(1, 0);

                double shoulderCurrent = ArmConstants.shoulderMotor.getCurrent(shoulderVelocity, shoulderVoltage);
                double elbowCurrent    = ArmConstants.elbowMotor.getCurrent(elbowVelocity, elbowVoltage);

                double shoulderTorque = ArmConstants.shoulderMotor.getTorque(shoulderCurrent);
                double elbowTorque    = ArmConstants.elbowMotor.getTorque(elbowCurrent);

                JointAccelerations accel = KinematicsDynamics.getAccelerations(
                    shoulderTorque, elbowTorque,
                    shoulderAngle, elbowAngle,
                    shoulderVelocity, elbowVelocity);

                return VecBuilder.fill(
                    shoulderVelocity, elbowVelocity,
                    accel.shoulderAcceleration(), accel.elbowAcceleration());
            },
            x, u, 0.020
        );

        return new SimState(
            nextState.get(0, 0), nextState.get(1, 0),
            nextState.get(2, 0), nextState.get(3, 0));
    }

    @Override
    public void updateInputs(ArmIOinputs inputs) {
        getNextState(currentState);
        inputs.elbowAngle = currentState.elbowAngle();
        inputs.elbowVelocity = currentState.elbowVelocity();
        inputs.shoulderAngle = currentState.shoulderAngle();
        inputs.shoulderVelocity = currentState.shoulderVelocity();
        inputs.elbowAppliedVoltage = appliedElbowVoltage;
        inputs.shoulderAppliedVoltage = appliedShoulderVoltage;

    }

    @Override
    public void setVoltage(double shoulderVoltage, double elbowVoltage) {
        double shoulderAppliedVoltage = shoulderVoltage;
        double elbowAppliedVoltage = elbowVoltage;
    }

}