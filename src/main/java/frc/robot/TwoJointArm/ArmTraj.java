package frc.robot.TwoJointArm;

import java.util.List;

public class ArmTraj {
    private final List<ArmPointTraj> waypoints;
    private final double duration;

    public ArmTraj(List<ArmPointTraj> waypoints, double duration) {
        this.waypoints = waypoints;
        this.duration = duration;


    }    public ArmPointTraj sample(double time) {
        if (time <= 0) {
            return waypoints.get(0);
        } else if (time >= duration) {
            return waypoints.get(waypoints.size() - 1);
        } else {
            return null; // keep working on that
        }

    
    }



}
