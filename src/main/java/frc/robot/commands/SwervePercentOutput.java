package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.SwerveModule;
import frc.robot.util.Vector;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

public class SwervePercentOutput extends IndefiniteCommand {

    public SwervePercentOutput() {
        requires(Drivetrain.getInstance());
    }

    @Override
    protected void execute() {
        double translateX = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.XBOX_JOYSTICK_DEADBAND);
        double translateY = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.XBOX_JOYSTICK_DEADBAND);
        double rotate = OI.getInstance().getDriverGamepad().getRightX();

        SwerveModule[] modules = new SwerveModule[4];

        modules[0] = Drivetrain.getInstance().getTopLeft();
        modules[1] = Drivetrain.getInstance().getTopRight();
        modules[2] = Drivetrain.getInstance().getBottomLeft();
        modules[3] = Drivetrain.getInstance().getBottomRight();

        Vector transVector = new Vector(translateX, translateY);

        for(int i = 0; i < modules.length; i++)
        {
            Vector vec = Drivetrain.rotationVectors[i];
            Vector rotateVector = new Vector(vec.getX(), vec.getY());
            rotateVector.scale(rotate);
            
            Vector finalVector = Vector.add(rotateVector, transVector);

            double newAngle = 360 - finalVector.getAngle();
            double magnitude = finalVector.getMagnitude();
            boolean invertDrives = modules[i].setTargetAngle(newAngle);
            if(invertDrives)
                magnitude = -magnitude;
            modules[i].getDriveMotor().set(ControlMode.PercentOutput, magnitude);
        }

    }
}