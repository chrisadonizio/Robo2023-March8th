package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class WheelDrive extends SubsystemBase{
private TalonSRX angleMotor;
private CANSparkMax driveMotor;
public PIDController anglePIDController;
private int counter = 0; 
public double PIDVal = 0;
public WheelDrive (int driveMotor, int angleMotor) {
    this.angleMotor = new TalonSRX (angleMotor);
    this.driveMotor = new CANSparkMax(driveMotor, MotorType.kBrushless);
    // this.angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    anglePIDController = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    anglePIDController.enableContinuousInput(-180, 180);
    anglePIDController.setTolerance(5, 10);
}
// public WheelDrive(){
//     anglePIDController = new PIDController(Constants.kP, Constants.kI, Constants.kD);
//     anglePIDController.enableContinuousInput(-180, 180);
//     anglePIDController.setTolerance(5, 10);
// }
// public double fakePID(double currentAngle, double desiredAngle){
//     return MathUtil.clamp(anglePIDController.calculate(currentAngle,desiredAngle),-0.8,0.8);
// }
public void drive (double speed, double Motorangle, double desiredAngle, boolean isForward) {
    if(isForward==false){
        driveMotor.set(speed*-0.3);
    }
    else{
    driveMotor.set(speed*0.3);
    }
    PIDVal = anglePIDController.calculate(Motorangle,desiredAngle)/180; 
    // if(PIDVal<0){
    //     PIDVal = Math.pow(PIDVal,0.8);
    //     PIDVal*=-1;
    // }
    // else{
    //     PIDVal = Math.pow(PIDVal,0.8);
    // }
    angleMotor.set(ControlMode.PercentOutput,(PIDVal));
}
public void moveAngleMotor(double currentPosition, double desiredPosition, boolean isForward){
    PIDVal = anglePIDController.calculate(currentPosition,desiredPosition);
    if(anglePIDController.atSetpoint())
    {
    if(isForward){
        angleMotor.set(ControlMode.PercentOutput,(PIDVal/180) * 0.3);
    }
    else{
        angleMotor.set(ControlMode.PercentOutput,(PIDVal/180*-1) * 0.3);
    }
    }
}
public void driveAngleMotor(double output){
	angleMotor.set(ControlMode.PercentOutput,output);
}
public void driveSpeedMotor(double output){
    driveMotor.set(output);	
}

public double getRawSensorVal(){
    return angleMotor.getSelectedSensorPosition();
}
public double getSensorValue(){
    double tempPos = angleMotor.getSelectedSensorPosition(0);
    double degrees = (2 * ((tempPos/1024) - Math.floor(1.0/2 + tempPos/1024)) * 180);
    return -degrees;
}
public void stopMotor(){
    angleMotor.set(ControlMode.PercentOutput, 0);
    driveMotor.set(0);
}


}