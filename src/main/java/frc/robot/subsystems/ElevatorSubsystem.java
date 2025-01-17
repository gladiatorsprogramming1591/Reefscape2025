package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem {
    
    private RelativeEncoder encoder;
    private SparkMax motor;
    private DigitalInput lowerLimit;
    private DigitalInput upperLimit;

    public ElevatorSubsystem() {
        motor = new SparkMax(ElevatorConstants.MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        lowerLimit = new DigitalInput(ElevatorConstants.LOWER_LIMIT_ID);
        upperLimit = new DigitalInput(ElevatorConstants.UPPER_LIMIT_ID);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public void setSpeed(double motorSpeed) {
        motor.set(motorSpeed);
    }

    public boolean isAtLimit() {
        return isAtLowerLimit() || isAtUpperLimit();
    }

    public boolean isAtLowerLimit() {
        return lowerLimit.get();
    }

    public boolean isAtUpperLimit() {
        return upperLimit.get();
    }
}
