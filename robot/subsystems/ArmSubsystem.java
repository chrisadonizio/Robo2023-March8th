package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class ArmSubsystem extends SubsystemBase {
private Spark arm_motor;
  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem(int channel)
  {      arm_motor = new Spark(channel);
  }

  public void ascend(){
    arm_motor.setInverted(false);
    arm_motor.set(.4);
  }

  public void descend(){
    arm_motor.setInverted(true);
    arm_motor.set(.4);
  }

  public void condescend(){
    arm_motor.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}