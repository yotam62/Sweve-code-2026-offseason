package frc.robot.drive;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {

    Pose2d oldposeLL4 = new Pose2d();
    Pose2d oldposeLL3GS = new Pose2d();
    Pose2d oldposeLL3GF = new Pose2d();

    
        StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
        .getStructTopic("limelight pose", Pose2d.struct).publish(); 

        public double time_LL3GS = 0;
        public Pose2d MT2pose_LL3GS = new Pose2d();
        public int tagCount_LL3GS = 0;    
        public double avgDistance_LL3GS = 0;
        public double rotation_LL3GS = 0.0;
        public boolean isNew_LL3GS = false;
        public boolean isConnected_LL3GS = false;
       

        public double time_LL4 = 0;
        public Pose2d MT2pose_LL4 = new Pose2d();
        public int tagCount_LL4 = 0;    
        public double avgDistance_LL4 = 0;
        public double rotation_LL4 = 0.0;
        public boolean isNew_LL4 = false;
        public boolean isConnected_LL4 = false; 

        public double time_LL3GF = 0;
        public Pose2d MT2pose_LL3GF = new Pose2d();
        public int tagCount_LL3GF = 0;    
        public double avgDistance_LL3GF = 0;
        public double rotation_LL3GF = 0.0;
        public boolean isNew_LL3GF = false;
        public boolean isConnected_LL3GF = false;

    //private Field2d field = new Field2d();


    DoubleArraySubscriber Limelight_4 = NetworkTableInstance.getDefault().getTable("limelight-four").getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[11], PubSubOption.keepDuplicates(true));
    DoubleArraySubscriber ll4_rotation = NetworkTableInstance.getDefault().getTable("limelight-four").getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[11], PubSubOption.keepDuplicates(true));
    DoubleSubscriber connected4 = NetworkTableInstance.getDefault().getTable("limelight-four").getDoubleTopic("tv").subscribe(-1, PubSubOption.keepDuplicates(true));

    DoubleArraySubscriber Limelight_3GS = NetworkTableInstance.getDefault().getTable("limelight-threegs").getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[11], PubSubOption.keepDuplicates(true));
    DoubleArraySubscriber ll3gs_rotation = NetworkTableInstance.getDefault().getTable("limelight-threegs").getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[11], PubSubOption.keepDuplicates(true));
    DoubleSubscriber connected3gs = NetworkTableInstance.getDefault().getTable("limelight-threegs").getDoubleTopic("tv").subscribe(-1, PubSubOption.keepDuplicates(true));

    DoubleArraySubscriber Limelight_3GF = NetworkTableInstance.getDefault().getTable("limelight-threegf").getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[11], PubSubOption.keepDuplicates(true));
    DoubleArraySubscriber ll3gf_rotation = NetworkTableInstance.getDefault().getTable("limelight-threegf").getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[11], PubSubOption.keepDuplicates(true));
    DoubleSubscriber connected3gf = NetworkTableInstance.getDefault().getTable("limelight-threegf").getDoubleTopic("tv").subscribe(-1, PubSubOption.keepDuplicates(true));

    public void periodic() {
   
        TimestampedDoubleArray data_LL4 = Limelight_4.getAtomic();
        TimestampedDoubleArray rotation_LL4 = ll4_rotation.getAtomic();

        double timestamp_LL4 = data_LL4.serverTime/1000000.0 - data_LL4.value[6]/1000.0;
        MT2pose_LL4 = new Pose2d(new Translation2d(data_LL4.value[0], data_LL4.value[1]), Rotation2d.fromDegrees(data_LL4.value[5]));
        avgDistance_LL4 = data_LL4.value[9];
    
        isNew_LL4 = !MT2pose_LL4.getTranslation().equals(oldposeLL4.getTranslation()) && data_LL4.value[7] > 0;
        oldposeLL4 = MT2pose_LL4;
        time_LL4 = timestamp_LL4;
        tagCount_LL4 = (int) data_LL4.value[7];
        rotation_LL4 = rotation_LL4.value[5];
        isConnected_LL4 = connected4.get() != -1;
        
        TimestampedDoubleArray data_LL3GS = Limelight_3GS.getAtomic();
        TimestampedDoubleArray rotation_LL3GS = ll3gs_rotation.getAtomic();

        publisher.set(new Pose2d(new Translation2d(rotation_LL3GS.value[0], rotation_LL3GS.value[1]), Rotation2d.fromDegrees(rotation_LL3GS.value[5])));

    
        double timestamp_LL3GS = data_LL3GS.serverTime/1000000.0 - data_LL3GS.value[6]/1000.0;
        MT2pose_LL3GS = new Pose2d(new Translation2d(data_LL3GS.value[0], data_LL3GS.value[1]), Rotation2d.fromDegrees(data_LL3GS.value[5]));
        avgDistance_LL3GS = data_LL3GS.value[9];
    
        isNew_LL3GS = !MT2pose_LL3GS.getTranslation().equals(oldposeLL3GS.getTranslation()) && data_LL3GS.value[7] > 0;
        oldposeLL3GS = MT2pose_LL3GS;
        time_LL3GS = timestamp_LL3GS;
        tagCount_LL3GS = (int) data_LL3GS.value[7];
        isConnected_LL3GS = connected3gs.get() != -1;
        rotation_LL3GS = rotation_LL3GS.value[5];

        TimestampedDoubleArray data_LL3GF = Limelight_3GF.getAtomic();
        TimestampedDoubleArray rotation_LL3GF = ll3gf_rotation.getAtomic();

        double timestamp_LL3GF = data_LL3GF.serverTime/1000000.0 - data_LL3GF.value[6]/1000.0;
        MT2pose_LL3GF = new Pose2d(new Translation2d(data_LL3GF.value[0], data_LL3GF.value[1]), Rotation2d.fromDegrees(data_LL3GF.value[5]));
        avgDistance_LL3GF = data_LL3GF.value[9];
        isNew_LL3GF = !MT2pose_LL3GF.getTranslation().equals(oldposeLL3GF.getTranslation()) && data_LL3GF.value[7] > 0;
        oldposeLL3GF = MT2pose_LL3GF;
        time_LL3GF = timestamp_LL3GF;
        tagCount_LL3GF = (int) data_LL3GF.value[7];
        isConnected_LL3GF = connected3gf.get() != -1;
        rotation_LL3GF = rotation_LL3GF.value[5];
        
    }
    
  
    
}
