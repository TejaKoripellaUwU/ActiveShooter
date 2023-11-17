package frc.robot.subsystems;

import frc.robot.constants.ShooterConstants;

public enum ShooterType{
    ROLLER(ShooterConstants.MOTOR_TO_ROLLER_GEAR_RATIO),
    FLYWHEEL(ShooterConstants.MOTOR_TO_FLWHEEL_GEAR_RATIO),
    SCHOOL(8008);

    public double mGearRatioToMotor;
    public double mGearRatioToSensor;
    private ShooterType(double gearRatioToMotor){
        mGearRatioToMotor = gearRatioToMotor;
        mGearRatioToSensor = 1/mGearRatioToMotor;
    }
}
