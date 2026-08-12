package frc.robot.TwoJointArm.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TwoJointArm.ArmPointTraj;
import frc.robot.TwoJointArm.ArmSubsystem;
import frc.robot.TwoJointArm.ArmTraj;
import frc.robot.TwoJointArm.KinematicsDynamics;

public class StayCommand extends Command {

    KinematicsDynamics kinematics = new KinematicsDynamics();
    private ArmSubsystem Arm;
    ArmPointTraj staticState;

        public StayCommand(ArmSubsystem Arm) {
        this.Arm = Arm;

        addRequirements(Arm);
    }

    public void initialize() {
        staticState = new ArmPointTraj(Arm.getcurrentState().shoulderAngle(), Arm.getcurrentState().elbowAngle(), 0, 0, 0, 0);
    }

    public void execute() {
        KinematicsDynamics.JointVoltages jointVoltages = kinematics.getNeededVoltage(staticState);
        double elbowVoltagePID = kinematics.addPID(jointVoltages, staticState, Arm.getcurrentState()).elbowVoltage();
        double shoulderVoltagePID = kinematics.addPID(jointVoltages, staticState, Arm.getcurrentState()).shoulderVoltage();

        Arm.setVoltage(shoulderVoltagePID, elbowVoltagePID);
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        Arm.setVoltage(0, 0);
    }
    
}
