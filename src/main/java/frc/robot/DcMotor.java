package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DCMotor extends SubsystemBase {
  private final TalonFX motor;

  public DCMotor(int motorID) {
    motor = new TalonFX(motorID);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public void stop() {
    motor.set(0);
  }
}