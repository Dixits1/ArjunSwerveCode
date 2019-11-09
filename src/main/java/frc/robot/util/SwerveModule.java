package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSTalon;

/**
 * A swerve module on the drivetrain.
 * A swerve module contains one motor to spin the wheel and another to change the wheel's angle.
 * 
 * @author Angela Jia
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @since 11/1/19
 */
public class SwerveModule {


    private static final double ENCODERTICKS = 1024.0;

    private static final double VOLTAGE_COMP = 10;

    private static final int CURRENT_CONTINUOUS = 40;
    private static final int CURRENT_PEAK = 60;
    private static final int CURRENT_PEAK_DUR = 500;

    // Motor inversions
    private final boolean DRIVE_INVERTED;
    private final boolean ANGLE_INVERTED;

    private final boolean DRIVE_SENSOR_PHASE;
    private final boolean ANGLE_SENSOR_PHASE;

    private HSTalon angleMotor;
    private HSTalon driveMotor;

    public SwerveModule(int driveId, boolean invertDriveTalon, boolean driveSensorPhase, int angleId, boolean invertAngleTalon, boolean angleSensorPhase) {
        driveMotor = new HSTalon(driveId);
        angleMotor = new HSTalon(angleId);

        DRIVE_INVERTED = invertDriveTalon;
        ANGLE_INVERTED = invertAngleTalon;

        DRIVE_SENSOR_PHASE = driveSensorPhase;
        ANGLE_SENSOR_PHASE = angleSensorPhase;

        swerveDriveInverted = false;

        driveTalonInit(driveMotor);
        angleTalonInit(angleMotor);
    }
    
    public void driveTalonInit(HSTalon talon) {
        talon.configFactoryDefault();
        
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        talon.setNeutralMode(NeutralMode.Brake);

        talon.setInverted(DRIVE_INVERTED);
        talon.setSensorPhase(DRIVE_SENSOR_PHASE);
        
        talon.configForwardSoftLimitEnable(false);
        talon.configReverseSoftLimitEnable(false);
        talon.overrideLimitSwitchesEnable(false);

        talon.setSelectedSensorPosition(0);

        configCurrentLimit(talon);

        talon.configVoltageCompSaturation(VOLTAGE_COMP);
        talon.enableVoltageCompensation(true);
    }

    public void angleTalonInit(HSTalon talon) {
        talon.configFactoryDefault();
        
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        talon.setNeutralMode(NeutralMode.Brake);

        talon.setInverted(ANGLE_INVERTED);
        talon.setSensorPhase(ANGLE_SENSOR_PHASE);

        talon.configForwardSoftLimitEnable(false);
        talon.configReverseSoftLimitEnable(false);
        talon.overrideLimitSwitchesEnable(false);

        configCurrentLimit(talon);

        talon.configVoltageCompSaturation(VOLTAGE_COMP);
        talon.enableVoltageCompensation(true);
    } 
    
    public static void configCurrentLimit(HSTalon talon) {
        talon.configContinuousCurrentLimit(CURRENT_CONTINUOUS);
        talon.configPeakCurrentLimit(CURRENT_PEAK);
        talon.configPeakCurrentDuration(CURRENT_PEAK_DUR);
        talon.enableCurrentLimit(true);
    }
    
    public HSTalon getAngleMotor() {
        return angleMotor;    
    }            
        
    public HSTalon getDriveMotor() {
        return driveMotor;
    }

    //the following code from here to the bottom has been stolen from team 364
    //https://github.com/teamfusion364/Swerve-MiniBot/
    //plz dont sue me thx
    public boolean setTargetAngle(double targetAngle) {
        boolean invertDrives;

        targetAngle = modulate360(targetAngle);
        double currentAngle = toDegrees(angleMotor.getSelectedSensorPosition(0));
        double currentAngleMod = modulate360(currentAngle);
        if (currentAngleMod < 0) currentAngleMod += 360;
        double delta = currentAngleMod - targetAngle;
        if (delta > 180) {
            targetAngle += 360;
        } else if (delta < -180) {
            targetAngle -= 360;
        }
        delta = currentAngleMod - targetAngle;
        if (delta > 90 || delta < -90) {
            if (delta > 90)
                targetAngle += 180;
            else if (delta < -90)
                targetAngle -= 180;
            invertDrives = false;
        } else {
            invertDrives = true;
        }

        targetAngle += currentAngle - currentAngleMod;
        targetAngle = toCounts(targetAngle);
        getAngleMotor().set(ControlMode.Position, targetAngle);

        return invertDrives;
    }

    public static double toCounts(double units){
        return units * ENCODERTICKS / 360.0;
    }

    public static double toDegrees(double units){
        return units * (360 / ENCODERTICKS);
    }

    public double modulate360(double units){
        return units %= 360;
    }
}