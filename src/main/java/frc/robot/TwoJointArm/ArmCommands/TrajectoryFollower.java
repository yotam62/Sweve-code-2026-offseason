package frc.robot.TwoJointArm.ArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TwoJointArm.ArmPointTraj;
import frc.robot.TwoJointArm.ArmSubsystem;
import frc.robot.TwoJointArm.ArmTraj;
import frc.robot.TwoJointArm.KinematicsDynamics;
import frc.robot.TwoJointArm.KinematicsDynamics.JointVoltages;


    
public class TrajectoryFollower extends Command {


    KinematicsDynamics kinematics = new KinematicsDynamics();
    private ArmSubsystem Arm;
    private ArmTraj traj;
    private Timer trajTimer = new Timer();

    public TrajectoryFollower(ArmTraj traj, ArmSubsystem Arm) {

        this.traj = traj;
        this.Arm = Arm;

        addRequirements(Arm);
    }

    public void initialize() {
        trajTimer.restart();
        trajTimer.start();

    }

    public void execute() {
        JointVoltages jointVoltages;
        double elbowVoltage;
        double shoulderVoltage;
        double Time = trajTimer.get();
        ArmPointTraj currentPoint = traj.sample(Time);
        jointVoltages = kinematics.getNeededVoltage(currentPoint);
        jointVoltages = kinematics.addPID(jointVoltages, currentPoint, Arm.getcurrentState());
        elbowVoltage = jointVoltages.elbowVoltage();
        shoulderVoltage = jointVoltages.shoulderVoltage();

        Arm.setVoltage(shoulderVoltage, elbowVoltage);


    }

    public boolean isFinished() {
        return trajTimer.get() >= traj.duration() && 
        (Math.abs((((traj.waypoints().get(traj.waypoints().size() -1)).elbowAngle()) - Arm.getcurrentState().elbowAngle())) < 0.03) &&
         (Math.abs((((traj.waypoints().get(traj.waypoints().size() -1)).shoulderAngle()) - Arm.getcurrentState().shoulderAngle())) < 0.03)&&
         (Math.abs((((traj.waypoints().get(traj.waypoints().size() -1)).elbowAngularVelocity()) - Arm.getcurrentState().elbowAngularVelocity())) < 0.1) &&
         (Math.abs((((traj.waypoints().get(traj.waypoints().size() -1)).shoulderAngularVelocity()) - Arm.getcurrentState().shoulderAngularVelocity())) < 0.1);

    }

    public void end(boolean interrupted) {
        JointVoltages jointVoltages = kinematics.getNeededVoltage(traj.waypoints().get(traj.waypoints().size() -1));
        double elbowVoltage = jointVoltages.elbowVoltage();
        double shoulderVoltage = jointVoltages.shoulderVoltage();
        Arm.setVoltage(shoulderVoltage, elbowVoltage);
        trajTimer.stop();
        trajTimer.reset();

    }

    

    
}
