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

public class DriveCommand extends Command {
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private Drive swerve;
    private DoubleSupplier rotationSupplier;
    CommandXboxController controller;
    
      public DriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, Drive swerve, CommandXboxController controller) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.swerve = swerve;
        this.rotationSupplier = rotationSupplier;
        this.controller = controller;
        addRequirements(swerve);


    }

    public void execute() {
        Translation2d linearVelocity;
       // if(controller.isConnected()){            
            linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());



        double omega = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.1);
 
        omega = Math.copySign(omega * omega, omega);
        boolean isFlipped =
            DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

        ChassisSpeeds speeds = new ChassisSpeeds(
         linearVelocity.getX() * swerve.getMaxLinearVelocityPerSecond(),
         linearVelocity.getY() * swerve.getMaxLinearVelocityPerSecond(),
         omega* swerve.getMaxAngularSpeedRadPerSecond());

        swerve.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
        speeds,
        isFlipped
            ? swerve.getRotation().plus(new Rotation2d(Math.PI))
            : swerve.getRotation()));

    } 
    //}

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
 }
 // continue building the DriveCommand and follow the video of the zoom