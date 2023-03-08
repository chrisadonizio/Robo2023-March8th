package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;


public class ElevatorSubsystem extends SubsystemBase {
private Talon elevator_motor1;
private Talon elevator_motor2;

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem(int channel, int channel2)
  {      elevator_motor1 = new Talon(channel);
         elevator_motor2 = new Talon(channel2);


  }

  public void ascend(){
        elevator_motor1.setInverted(false);
        elevator_motor1.set(.4);
        elevator_motor2.setInverted(true);
        elevator_motor2.set(.4);
  }

  public void descend(){
        elevator_motor1.setInverted(true);
        elevator_motor1.set(.4);
        elevator_motor2.setInverted(true);
        elevator_motor2.set(.4);
  }

  public void condescend(){
      elevator_motor1.stopMotor();
      elevator_motor2.stopMotor();

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