package java.frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  public CANSparkMax shooterMotor1 = new CANSparkMax(Constants.shooterMotor1_ID, MotorType.kBrushless);
  public CANSparkMax shooterMotor2 = new CANSparkMax(Constants.shooterMotor2_ID, MotorType.kBrushless);
  
  shooterMotor2.follow(shooterMotor1);
  
  @Override
  public void periodic() {
    int[] motorSpeeds = {.1, .2, .4, .5, .7, .8, 1};
    
    if (operatorStick.getRawButton(2)) {
      for (i = 0; i < motorSpeeds.length; i++) {
        shooterMotor1.set(i);
        wait(715);
      }
    }
  }
}
