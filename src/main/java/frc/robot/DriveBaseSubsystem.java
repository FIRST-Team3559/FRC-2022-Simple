package frc.robot;

import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveBaseSubsystem extends SubsystemBase {
   // Motor controllers Left
  public static CANSparkMax mc_leftFront = new CANSparkMax(Constants.FL_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
  public CANSparkMax mc_leftRear = new CANSparkMax(Constants.RL_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
  public MotorControllerGroup mcg_left = new MotorControllerGroup(mc_leftFront, mc_leftRear);
  // Motor controllers Right
  public static CANSparkMax mc_rightFront = new CANSparkMax(Constants.FR_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
  public CANSparkMax mc_rightRear = new CANSparkMax(Constants.RR_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
  public MotorControllerGroup mcg_right = new MotorControllerGroup(mc_rightFront, mc_rightRear);
  
  public DifferentialDrive drivetrain = new DifferentialDrive(mcg_left, mcg_right);
  public final static SparkMaxRelativeEncoder m_leftEncoder = new SparkMaxRelativeEncoder(mc_leftFront, null, 0);
  private final static SparkMaxRelativeEncoder m_rightEncoder = new SparkMaxRelativeEncoder(mc_rightFront, null, 0);
  public final ADXRS450_Gyro gyro;
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Constants.kTrackwidthMeters);
  public static Rotation2d gyroAngle = new Rotation2d();
  public static DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(gyroAngle);
  public static Pose2d m_pose;
  
  /** Creates a new DriveSubsystem. 
   * @return */
  public void DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    
    resetEncoders();
  }
   
  @Override
public static void periodic() {
  ADXRS450_Gyro m_gyro;
  // Get my gyro angle. We are negating the value because gyros return positive
  // values as the robot turns clockwise. This is not standard convention that is
  // used by the WPILib classes.
  gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle());

  // Update the pose
  m_pose = m_odometry.update(gyroAngle, m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
}
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  // Resets the drive encoders to currently read a position of 0.
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  // Gets the average distance of the two encoders.
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  public SparkMaxRelativeEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public SparkMaxRelativeEncoder getRightEncoder() {
    return m_rightEncoder;
  }
  
  public void manualDrive(double getLeftStick, double getRightStick) {
    drivetrain.tankDrive(getLeftStick, getRightStick);
  }
}
