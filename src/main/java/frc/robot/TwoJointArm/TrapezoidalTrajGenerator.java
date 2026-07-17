package frc.robot.TwoJointArm;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Constants;

public class TrapezoidalTrajGenerator implements TrajGeneratorInterface {


    private record JointProfile(
        double startAngle,
        double goalAngle,
        double direction,
        double maxVelocity,
        double maxAcceleration,
        double maxDeceleration,
        double accelTime,
        double cruiseTime,
        double decelTime,
        double totalTime
    ) {}

    private record JointState(double angle, double velocity, double acceleration) {}

    private JointProfile computeProfile(double start, double goal, double maxVel, double maxAccel, double maxDecel) {
        double distance = Math.abs(goal - start);
        double direction = Math.signum(goal - start);

        double accelDistance = (maxVel * maxVel) / (2 * maxAccel);
        double decelDistance = (maxVel * maxVel) / (2 * maxDecel);

        double accelTime;
        double decelTime;
        double cruiseTime;

        if (accelDistance + decelDistance > distance) {

            maxVel = Math.sqrt((2 * distance * maxAccel * maxDecel) / (maxAccel + maxDecel));
            accelTime = maxVel / maxAccel;
            decelTime = maxVel / maxDecel;
            cruiseTime = 0;
        } else {
            double cruiseDistance = distance - accelDistance - decelDistance;
            accelTime = maxVel / maxAccel;
            decelTime = maxVel / maxDecel;
            cruiseTime = cruiseDistance / maxVel;
        }

        double totalTime = accelTime + cruiseTime + decelTime;

        return new JointProfile(start, goal, direction, maxVel, maxAccel, maxDecel, accelTime, cruiseTime, decelTime, totalTime);
    }

    private JointState sampleJoint(JointProfile p, double t) {
        t = Math.min(t, p.totalTime());

        if (t < p.accelTime()) {
            double angle = p.startAngle() + p.direction() * 0.5 * p.maxAcceleration() * t * t;
            double velocity = p.direction() * p.maxAcceleration() * t;
            double acceleration = p.direction() * p.maxAcceleration();
            return new JointState(angle, velocity, acceleration);
        }

        if (t < p.accelTime() + p.cruiseTime()) {
            double accelDistance = 0.5 * p.maxAcceleration() * p.accelTime() * p.accelTime();
            double angle = p.startAngle() + p.direction() * (accelDistance + p.maxVelocity() * (t - p.accelTime()));
            double velocity = p.direction() * p.maxVelocity();
            return new JointState(angle, velocity, 0);
        }

        double timeRemaining = p.totalTime() - t;
        double angle = p.goalAngle() - p.direction() * 0.5 * p.maxDeceleration() * timeRemaining * timeRemaining;
        double velocity = p.direction() * p.maxDeceleration() * timeRemaining;
        double acceleration = -p.direction() * p.maxDeceleration();
        return new JointState(angle, velocity, acceleration);
    }

    @Override
    public ArmTraj createTraj(ArmPointTraj start, ArmPointTraj goal, List<Obstacle> obstacle) {
        JointProfile shoulderProfile = computeProfile(
            start.shoulderAngle(), goal.shoulderAngle(),
            Constants.ArmConstants.shoulderMaxVelocity,
            Constants.ArmConstants.shoulderMaxAcceleration,
            Constants.ArmConstants.shoulderMaxDeceleration
        );

        JointProfile elbowProfile = computeProfile(
            start.elbowAngle(), goal.elbowAngle(),
            Constants.ArmConstants.elbowMaxVelocity,
            Constants.ArmConstants.elbowMaxAcceleration,
            Constants.ArmConstants.elbowMaxDeceleration
        );


        double totalTime = Math.max(shoulderProfile.totalTime(), elbowProfile.totalTime());

        List<ArmPointTraj> pointList = new ArrayList<>();
        List<Double> timeStamps = new ArrayList<>();

        for (double t = 0; t < totalTime; t += Constants.ArmConstants.dt) {
            JointState shoulder = sampleJoint(shoulderProfile, t);
            JointState elbow = sampleJoint(elbowProfile, t);

            pointList.add(new ArmPointTraj(
                shoulder.angle(), elbow.angle(),
                shoulder.velocity(), elbow.velocity(),
                shoulder.acceleration(), elbow.acceleration()
            ));
            timeStamps.add(t);
        }


        pointList.add(new ArmPointTraj(goal.shoulderAngle(), goal.elbowAngle(), 0, 0, 0, 0));
        timeStamps.add(totalTime);

        return new ArmTraj(pointList, totalTime, timeStamps);
    }
}