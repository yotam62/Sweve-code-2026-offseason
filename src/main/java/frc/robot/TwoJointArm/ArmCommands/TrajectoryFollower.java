package frc.robot.TwoJointArm.ArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TwoJointArm.ArmPointTraj;
import frc.robot.TwoJointArm.ArmSubsystem;
import frc.robot.TwoJointArm.ArmTraj;
import frc.robot.TwoJointArm.KinematicsDynamics;
import frc.robot.TwoJointArm.KinematicsDynamics.JointVoltages;


    
public class TrajectoryFollower extends Command {


    
    private ArmSubsystem Arm;
    private ArmTraj traj;
    private Timer trajTimer = new Timer();
    int Point;

    public TrajectoryFollower(ArmTraj traj, ArmSubsystem Arm) {
        this.traj = traj;
        this.Arm = Arm;

        addRequirements(Arm);
    }

    public void initialize() {
        trajTimer.start();
        Point = 0;

    }

    public void execute() {
        JointVoltages jointVoltages;
        double elbowVoltage;
        double shoulderVoltage;
        double Time = trajTimer.getTimestamp();
        ArmPointTraj currentPoint = traj.sample(Time);
        jointVoltages = KinematicsDynamics.getNeededVoltage(currentPoint);
        elbowVoltage = jointVoltages.elbowVoltage();
        shoulderVoltage = jointVoltages.shoulderVoltage();

        Arm.setVolatge(shoulderVoltage, elbowVoltage);


    }

    

    
}
