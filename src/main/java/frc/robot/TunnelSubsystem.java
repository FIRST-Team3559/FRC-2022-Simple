package java.frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TunnelSubsystem extends SubsystemBase {
  public Spark bottomTunnelMotor = new Spark(Constants.bottomTunnelChannel);
  public Spark topTunnelMotor = new Spark(Constants.topTunnelChannel);
  
  public void tunnel() {
    if (operatorStick.getRawButton(3)) {
      ballTunnel.set(-.7);
    } else {
      ballTunnel.set(0);
   }
  }
}
