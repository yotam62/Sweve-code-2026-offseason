package frc.robot.drive;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModuleState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;
import static edu.wpi.first.units.Units.*;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.drive.AutoAlignHubCommand;
import frc.robot.drive.VisionSubsystem.VisionMeasurement;

public class Drive extends SubsystemBase{

    static final double ODOMETRY_FREQUENCY = 150;
    private final Field2d m_field = new Field2d();
    public Rotation2d simRotation = new Rotation2d();
    public Rotation2d rawGyroRotation = new Rotation2d();
    
    private Twist2d twist = new Twist2d();
    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    private SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

    

    SwerveModuleState[] mods = new SwerveModuleState[] {
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState()};

    public SwerveModulePosition[] lastModulePositions = // For delta tracking
    new SwerveModulePosition[] {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()};
    static final Lock odometryLock = new ReentrantLock();

    private SwerveDriveKinematics kinematics =  SwerveConstants.swerveKinematics;


    public Drive(Module flModule, Module blModule, Module brModule, Module frModule) {  
        modules[0] = flModule;
        modules[1] = blModule;
        modules[2] = brModule;
        modules[3] = frModule;

        SmartDashboard.putData("field",m_field);
    }
    public double getMaxLinearVelocityPerSecond() {
        return 5.0;
    }
    public double getMaxAngularSpeedRadPerSecond() {
        return getMaxLinearSpeedMetersPerSec() * 1 / SwerveConstants.DRIVE_BASE_RADIUS;
    }





    public SwerveDrivePoseEstimator SwervePoseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d(0, 0, new Rotation2d()), VecBuilder.fill(0.005,0.005, Radians.convertFrom(5, Degrees)), VecBuilder.fill(0.05, 0.05, 999999999) );
    private boolean wasGyroDisconnected = false;
    SysIdRoutine routine;

    public Rotation2d getRotation() {
    return SwervePoseEstimator.getEstimatedPosition().getRotation();
    }   

    public void runVelocity(ChassisSpeeds speeds) {

    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02); // making sure the robot will move correctly with the spin and speed we give it. ALWAYS use it.
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds); //turning the robot spin and speed into the spin and speed of each swerve module.
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, getMaxLinearSpeedMetersPerSec()); // making sure each sweerve module doesn't  go over the speed limit, could slow down the robot.
    for (int i = 0; i < 4; i++) {
        modules[i].runSetpoint(setpointStates[i]);
    }


 }

    public double getMaxLinearSpeedMetersPerSec() {
        return 5.0;
    }



    public void periodic(){

    for (var module : modules) {
        module.periodic();
    }


    double[] sampleTimestamps =
    modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
 
 
    for (int i = 0; i < sampleCount; i++) {
    // Read wheel positions and deltas from each module
    // double vx = 0;
    // double vy = 0;
    modulePositions = new SwerveModulePosition[4];
    moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
    modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];//make an array
    moduleDeltas[moduleIndex] =
    new SwerveModulePosition(
    modulePositions[moduleIndex].distanceMeters
    - lastModulePositions[moduleIndex].distanceMeters,
    modulePositions[moduleIndex].angle);
    lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }
    
    twist = kinematics.toTwist2d(moduleDeltas);
    

    // Use the angle delta from the kinematics and module deltas
    Twist2d twist = kinematics.toTwist2d(moduleDeltas);
    rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta)); //TODo: change this to the if else
    
    
    

    
    //visionLock.lock();
    SwervePoseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    // visionLock.unlock();
    }
 


    

         m_field.setRobotPose(SwervePoseEstimator.getEstimatedPosition());
        
        


        }
 


    public Pose2d getPose() {
        return SwervePoseEstimator.getEstimatedPosition();
    }  
    public void resetPosition(Pose2d pose) {
        SwervePoseEstimator.resetPosition(rawGyroRotation, lastModulePositions, pose);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

     public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
        modules[i].runCharacterization(output);
        }
    }

     /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
        output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }


    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        double totalSpeed = 0;
        for (int i = 0; i < 4; i++) {
        states[i] = modules[i].getState();
        totalSpeed += states[i].speedMetersPerSecond;
        }
        
        return states;
    }

    public void addVision(VisionMeasurement measurement) {
    Vector<N3> stds = VecBuilder.fill(measurement.std()[0], measurement.std()[1], 9999999); /*  
 makes a "certainty vector" that is baisically how much we trust the vision measurement.
  //The first two numbers are the standard deviations of the x and y position, 
  and the last number is the standard deviation of the rotation. The lower the error, the more we trast vison, and vice versa */

 //visionLock.lock();
    SmartDashboard.putBoolean("angle conditions", Math.abs(gyroInputs.rollDegrees) < 1 || Math.abs(gyroInputs.pitchDegrees) < 1);
 if (Math.abs(gyroInputs.rollDegrees) < 5 && Math.abs(gyroInputs.pitchDegrees) < 5 && getGyroSpeed() < 180 && getTranslationalSpeed() < 5) {

 SwervePoseEstimator.addVisionMeasurement(new Pose2d(measurement.pose().getTranslation(), getRotation()), measurement.timestamp(), stds);

 if (gyroResetTimer.hasElapsed(15) && getGyroSpeed() < 1 && getTranslationalSpeed() < 0.1 && measurement.numTags() >= 2 && measurement.avgDistance() < 2.35) {
 SwervePoseEstimator.resetRotation(Rotation2d.fromDegrees(measurement.rotationDegreees()));
 //SmartDashboard.putBoolean("gyro reset", true);
 gyroResetTimer.restart();
 }
 /// SmartDashboard.putBoolean("gyro reset", false);

 

 }
 
 //visionLock.unlock();
 }

}