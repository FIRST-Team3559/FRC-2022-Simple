// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.Arrary;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.commands.ManualDriveCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private DifferentialDrive driveBase;
  private Joystick driverLeft;
  private Joystick driverRight;
  private Joystick operatorStick;
  private static final int leftLeaderDeviceID = 10;
  private static final int leftFollowerDeviceID = 11;
  private static final int rightLeaderDeviceID = 12;
  private static final int rightFollowerDeviceID = 13;
  private CANSparkMax leftLeader, leftFollower, rightLeader, rightFollower;
  private Spark feederMotor;
  private Spark ballTunnel;
  private Spark shooterMotor;
  private Thread m_visionThread;


  /**
   * 
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    leftLeader = new CANSparkMax(leftLeaderDeviceID, MotorType.kBrushless);
    leftFollower = new CANSparkMax(leftFollowerDeviceID, MotorType.kBrushless);
    rightLeader = new CANSparkMax(rightLeaderDeviceID, MotorType.kBrushless);
    rightFollower = new CANSparkMax(rightFollowerDeviceID, MotorType.kBrushless);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    feederMotor = new Spark(0);
    ballTunnel = new Spark(1);
    shooterMotor = new Spark(3);

    driveBase = new DifferentialDrive(leftLeader, rightLeader);

    driveLeft = new Joystick(0);
    driveRight = new Joystick(1);
    operatorStick = new Joystick(2);
    
    bottomTunnelMotor.enableDeadbandElimination(true);
    topTunnelMotor.enableDeadbandElimination(true);

    mc_leftRear.follow(mc_leftFront);
    mc_rightRear.follow(mc_rightFront);
    
    DriveSubsystem.getRightEncoder.setPosition(0);
    DriveSubsystem.getLeftEncoder.setPosition(0);

    if(leftLeader.setOpenLoopRampRate(.5) !=REVLibError.kOk) {
      SmartDashboard.putString("Ramp Rate", "Error");
    }

    if(leftFollower.setOpenLoopRampRate(.5) !=REVLibError.kOk) {
      SmartDashboard.putString("Ramp Rate", "Error");
    }

    if(rightLeader.setOpenLoopRampRate(.5) !=REVLibError.kOk) {
      SmartDashboard.putString("Ramp Rate", "Error");
    }

    if(rightFollower.setOpenLoopRampRate(.5) !=REVLibError.kOk) {
      SmartDashboard.putString("Ramp Rate", "Error");
    }
    CameraServer.startAutomaticCapture;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    
    /* Creates a thread which converts color images into grayscale,
    and then detects circle shapes which the robot will go to */
    m_visionThread = new Thread(
      () -> {
        // Starts the camera and sets the resolution, or frame size
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(640, 480);
        /* Initializes a sink and allows the Mat to access 
        camera images from the sink */
        CvSink cvSink = CameraServer.getVideo();
        CvSource outputStream = CameraServer.putVideo("Circle", 640, 480);
        Mat mat = new Mat();
        while (!Thread.interrupted()) {
                /* Tell the CvSink to grab a frame from the camera and put it
                in the source mat.  If there is an error notify the output */
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
                Imgproc.cvtColor(mat, mat, COLOR_BGR2GRAY, 3);
                Imgproc.HoughCircles(mat, mat, mat.HOUGH_GRADIENT, 1, 45, 75, 40, 20, 80);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
              }
            });
    
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    driveBase.tankDrive(-driverGamepad.getRawAxis(1), driverGamepad.getRawAxis(5));
    feeder();
    tunnel();
    shooter();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public void feeder() {
    if (operatorStick.getRawButton(5)) {
      feederMotor.set(0.5);
    } else {
    if (operatorStick.getRawButton(4)) {
      feederMotor.set(-.5);
    } else {
      feederMotor.set(0);
    }
  }
  public void tunnel() {
    if (operatorStick.getRawButton(3)) {
      ballTunnel.set(-.7);
    } else {
      ballTunnel.set(0);
   }
  }
  public void shooter() {
    int[] motorSpeeds = {.1, .2, .4, .5, .7, .8, 1};
    
    if (operatorStick.getRawButton(2)) {
      for (i = 0; i < motorSpeeds.length; i++) {
        shooterMotor.set(i);
        wait(715);
      }
    }
  }
}
