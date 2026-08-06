package frc.robot.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class ShootingPathsAuto extends Command{

    Drive swerve;
    Timer timer = new Timer();
    double TimeToFinish = 4.25;
    Translation2d velocityVector;
    Translation2d targetPosition;
    double rotationKP = 8;

    public ShootingPathsAuto(Drive swerve ,Translation2d targetPosition) {
        this.swerve = swerve;
        this.targetPosition = targetPosition;
        addRequirements(swerve);
    }

    public void initialize() {
        timer.restart();

        Translation2d currentPos = swerve.SwervePoseEstimator.getEstimatedPosition().getTranslation();
       
        Translation2d distance = targetPosition.minus(currentPos);
        double speed = currentPos.getDistance(targetPosition) / TimeToFinish;
        if(speed > 3) {
            speed = 3;
        }
        if(distance.getNorm() > 0.01) {
            velocityVector = distance.times(speed/distance.getNorm()); // normalize the distance vector and then scale it to the desired speed
        }
        else {
            velocityVector = new Translation2d();
        }

    }

    public void execute() {
        boolean isAllianceRed = DriverStation.getAlliance().isPresent() 
        && DriverStation.getAlliance().get() == Alliance.Red;
        Translation2d robotLocation = swerve.SwervePoseEstimator.getEstimatedPosition().getTranslation();
        Translation2d rotationVector = getHubLocation(isAllianceRed).minus(robotLocation);
        Rotation2d allignDirection = rotationVector.getAngle(); //all that shit just for allign direction

        Rotation2d currentAngle = swerve.SwervePoseEstimator.getEstimatedPosition().getRotation(); //just getting the omega (rotation)
        Rotation2d wantedAngle = allignDirection;    
        Rotation2d currentToWanted = wantedAngle.minus(currentAngle);
        double RadianDistance = currentToWanted.getRadians();
        RadianDistance = MathUtil.angleModulus(RadianDistance);
        double omega = RadianDistance*rotationKP;


        ChassisSpeeds speeds = new ChassisSpeeds
        (velocityVector.getX(),
        velocityVector.getY(),
        omega);
        boolean isFlipped =
        DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

        swerve.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      swerve.getRotation()));


    }

    public boolean isFinished() {
        return timer.hasElapsed(TimeToFinish+0.5);

    }
    public void end(boolean interupted){
        swerve.runVelocity(new ChassisSpeeds());

    }
    public Translation2d getHubLocation(boolean isAllianceRed){ 
        if(isAllianceRed){
            return new Translation2d(11.813388,4.034);
        } 
        else {
            return new Translation2d(4.626, 4.034);
        }
    }
}
