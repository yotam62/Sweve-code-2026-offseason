package frc.robot;

import frc.robot.lib.BLine.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drive.*;
import frc.robot.Constants.SwerveConstants;

public class RobotContainer {
  public final Drive drive;
  private FollowPath.Builder pathBuilder; // ← not final, initialized in constructor

  CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    if (Robot.isReal()) {
      drive = new Drive(
        new ModuleReal(SwerveConstants.Mod0.constants, 0), //fl
        new ModuleReal(SwerveConstants.Mod1.constants, 1), //bl
        new ModuleReal(SwerveConstants.Mod2.constants, 2), //br
        new ModuleReal(SwerveConstants.Mod3.constants, 3)); //fr

    }

    else {
      
          drive = new Drive(
      new ModuleSim(0, SwerveConstants.SwerveModuleConstants),
      new ModuleSim(1, SwerveConstants.SwerveModuleConstants),
      new ModuleSim(2, SwerveConstants.SwerveModuleConstants),
      new ModuleSim(3, SwerveConstants.SwerveModuleConstants));

    } 
    configureBindings();
  }


  




   private void configureBindings() {
    drive.setDefaultCommand(
      new DriveCommand(
        () -> -controller.getLeftX(),
        () -> -controller.getLeftY(),
        () -> -controller.getRightX(),
        drive,
        controller
      )); 

      controller.x().onTrue(FeedforwardCharacterization.feedforwardCommand(drive, controller.getHID()));
    }
  //   pathBuilder = new FollowPath.Builder(
  //       drive,
  //       drive::getPose,
  //       drive::getChassisSpeeds,
  //       drive::runVelocity,
  //       new PIDController(5.0, 0.0, 0.0),
  //       new PIDController(3.5, 0.0, 0.0),
  //       new PIDController(0.0, 0.0, 0.0)
  //   )
  //   .withDefaultShouldFlip();    


  //   configureBindings();
  // }

  // private void configureBindings() {
  //   drive.setDefaultCommand(
  //     new DriveCommand(
  //       () -> -controller.getLeftX(),
  //       () -> -controller.getLeftY(),
  //       () -> -controller.getHID().getRawAxis(2),
  //       drive,
  //       controller
  //     ));

  //   controller.x().whileTrue(new AutoAlignHubCommand(drive, 
  //       () -> -controller.getLeftX(), 
  //       () -> -controller.getLeftY(), 
  //       () -> -controller.getHID().getRawAxis(2), 
  //       controller));

  //   controller.x().whileTrue(new InstantCommand(() -> SmartDashboard.putBoolean("X pressed", true)))
  //       .onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("X pressed", false)));
  // }


  // public Command getAutonomousCommand() {

  //       boolean isAllianceRed = DriverStation.getAlliance().isPresent() 
  //       && DriverStation.getAlliance().get() == Alliance.Red;

    
  //   Path pathA = new Path("practice_pathA");
  //   Path pathB = new Path("practice_pathB");
  //   Path tuningPID = new Path("tuning PID");


  //   Pose2d pathBSStart = isAllianceRed? flipPose(pathB.getStartPose()) : pathB.getStartPose();
  //   Pose2d pathASS = isAllianceRed? flipPose(pathA.getStartPose()) : pathA.getStartPose();




  //   return Commands.sequence(

  //       new InstantCommand(() -> drive.resetPosition(pathASS)), // only reset once at start
  //       //pathBuilder.build(tuningPID)
  //       pathBuilder.build(pathA),
  //       new ShootingPathsAuto(drive, pathBSStart.getTranslation()),
  //       pathBuilder.build(pathB)
  //   );
  // }
  
  //   private Pose2d flipPose(Pose2d pose) {
    
  //     return new Pose2d(
  //       16.541 - pose.getX(),
  //       8.21 -pose.getY(),
  //       pose.getRotation().times(-1));
  //   }
  public Command getAutonomousCommand() {
    return Commands.none();
  }

}