package frc.robot.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*The plan:
 * Get the robot current location, the alliance color and the hub location in feild coordinates. 
 * make a vector that goes from the robot position to the hub position
 * make a  rotation2d object that is the  diraction we need to face (not nececerily rotation 2d)
 * make a chassisSpeeds object and also optimize the rotation.
 * check by the DriveCommand what needs to be done here and what willl happen in the drive class
 */

public class AutoAlignHubCommand extends Command {

    private Drive swerve;
    private DoubleSupplier rotationSupplier;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    boolean isAllianceRed;
    private boolean isHubLocationKnown = false;
    CommandXboxController controller;
    Translation2d robotLocation;
    Translation2d Hublocation;
    private double rotationKP = 8;
    double omega;

    
     
    public AutoAlignHubCommand(Drive swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, CommandXboxController controller) {
        this.swerve = swerve;
        this.rotationSupplier = rotationSupplier;
        this.controller = controller;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
            addRequirements(swerve);
    }


    public void initialize() {

        if(DriverStation.getAlliance().isPresent()){
            boolean isAllianceRed = DriverStation.getAlliance().isPresent() 
            && DriverStation.getAlliance().get() == Alliance.Red;
            isHubLocationKnown = true;
            Hublocation = getHubLocation(isAllianceRed);
        }
        
        else{
            isHubLocationKnown = false;
        }

    }
    public void execute() {


        if(!isHubLocationKnown) return ;
        
        if(!isHubLocationKnown && DriverStation.getAlliance().isPresent()){
            Hublocation = getHubLocation(isAllianceRed);
            isAllianceRed = DriverStation.getAlliance().get() == Alliance.Red;
            isHubLocationKnown = true;

        }


        calculateOmegaToHub(swerve.getPose().getTranslation(), swerve);

        Translation2d linearVelocity;
        linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
        if(omega > swerve.getMaxAngularSpeedRadPerSecond()){
            omega = swerve.getMaxAngularSpeedRadPerSecond();
        }
        else if(omega < -swerve.getMaxAngularSpeedRadPerSecond()){
            omega = -swerve.getMaxAngularSpeedRadPerSecond();
        }

        boolean isFlipped =
        DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
 
        ChassisSpeeds speeds = new ChassisSpeeds(
        linearVelocity.getX() * swerve.getMaxLinearVelocityPerSecond(),
        linearVelocity.getY() * swerve.getMaxLinearVelocityPerSecond(),
        omega);

        swerve.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped
                ? swerve.getRotation().plus(new Rotation2d(Math.PI))
                : swerve.getRotation()));


    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) { 
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    }
    public Translation2d getHubLocation(boolean isAllianceRed){ 
        if(isAllianceRed){
            return new Translation2d(11.813388,4.034);
        } 
        else {
            return new Translation2d(4.626, 4.034);
        }
    }

        public double calculateOmegaToHub(Translation2d robotLocation, Drive swerve){
        robotLocation = swerve.SwervePoseEstimator.getEstimatedPosition().getTranslation();
        Translation2d rotationVector = Hublocation.minus(robotLocation);
        Rotation2d allignDirection = rotationVector.getAngle();
        double error = allignDirection.minus(swerve.SwervePoseEstimator.getEstimatedPosition().getRotation()).getRadians();
        double radianDistance = MathUtil.angleModulus(error);
        double omega = radianDistance * rotationKP; 
        return omega;

        }


    
} 

