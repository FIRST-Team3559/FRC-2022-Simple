package java.frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TunnelSubsystem extends SubsystemBase {
  public Spark bottomTunnelMotor = new Spark(Constants.bottomTunnelChannel);
  public Spark topTunnelMotor = new Spark(Constants.topTunnelChannel);
  
}
