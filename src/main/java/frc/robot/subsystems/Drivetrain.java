package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.SwervePercentOutput;
import frc.robot.commands.SwerveTest;
import frc.robot.util.SwerveModule;
import frc.robot.util.Vector;

/**
 * Simulates the drivetrain subsystem on the robot. 
 * A Swerve Drivetrain contains four SwerveModules.
 * 
 * Acronyms:
 *      TL: Top Left
 *      TR: Top Right
 *      BL: Bottom Left
 *      BR: Bottom Right
 * 
 * @author Angela Jia
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @since 11/1/19
 */
public class Drivetrain extends Subsystem {

	public static Drivetrain instance;

	public static Vector[] rotationVectors;

	public static final double DT_LENGTH = 50;
	public static final double DT_WIDTH = 50;

    private SwerveModule topLeft;
    private SwerveModule topRight;
    private SwerveModule bottomLeft;
	private SwerveModule bottomRight;

	private static final boolean TL_DRIVE_INVERTED = false;
	private static final boolean TL_ANGLE_INVERTED = true;
	private static final boolean TR_DRIVE_INVERTED = false;
	private static final boolean TR_ANGLE_INVERTED = false;
    private static final boolean BL_DRIVE_INVERTED = true;
	private static final boolean BL_ANGLE_INVERTED = false;
	private static final boolean BR_DRIVE_INVERTED = false;
	private static final boolean BR_ANGLE_INVERTED = false;

	private static final boolean TL_DRIVE_SENSOR_PHASE = true;
	private static final boolean TL_ANGLE_SENSOR_PHASE = false;
	private static final boolean TR_DRIVE_SENSOR_PHASE = true;
	private static final boolean TR_ANGLE_SENSOR_PHASE = true;
    private static final boolean BL_DRIVE_SENSOR_PHASE = true;
	private static final boolean BL_ANGLE_SENSOR_PHASE = true;
	private static final boolean BR_DRIVE_SENSOR_PHASE = true;
	private static final boolean BR_ANGLE_SENSOR_PHASE = true;

	private Drivetrain() {
		topLeft = new SwerveModule(RobotMap.TL_DRIVE_ID, TL_DRIVE_INVERTED, TL_DRIVE_SENSOR_PHASE, RobotMap.TL_ANGLE_ID, TL_ANGLE_INVERTED, TL_ANGLE_SENSOR_PHASE);
		topRight = new SwerveModule(RobotMap.TR_DRIVE_ID, TR_DRIVE_INVERTED, TR_DRIVE_SENSOR_PHASE, RobotMap.TR_ANGLE_ID, TR_ANGLE_INVERTED, TR_ANGLE_SENSOR_PHASE);
		bottomLeft = new SwerveModule(RobotMap.BL_DRIVE_ID, BL_DRIVE_SENSOR_PHASE, BL_DRIVE_INVERTED, RobotMap.BL_ANGLE_ID, BL_ANGLE_INVERTED, BL_ANGLE_SENSOR_PHASE);
		bottomRight = new SwerveModule(RobotMap.BR_DRIVE_ID, BR_DRIVE_INVERTED, BR_DRIVE_SENSOR_PHASE, RobotMap.BR_ANGLE_ID, BR_ANGLE_INVERTED, BR_ANGLE_SENSOR_PHASE);

		initRotationVectors();
	}

	public SwerveModule getTopLeft() {
		return topLeft;
	}

	public SwerveModule getTopRight() {
		return topRight;
	}

	public SwerveModule getBottomLeft() {
		return bottomLeft;
	}

	public SwerveModule getBottomRight() {
		return bottomRight;
	}
	
	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new SwervePercentOutput());
	}
	/***
	 * 
	 */
	public void setDrivetrain(Vector tr, Vector tl, Vector br, Vector bl) {
		
		// Scale down the vectors so that the largest magnitude is at the maximum speed
		double largestMag = max4(tr.getMagnitude(), tl.getMagnitude(), br.getMagnitude(), bl.getMagnitude());
		
		if(largestMag < 1) 
			largestMag = 1;

		tr.scale(1 / largestMag);
		tl.scale(1 / largestMag);
		br.scale(1 / largestMag);
		bl.scale(1 / largestMag);

		// topRight.setVector(tr.getMagnitude());
		// topLeft.setVector(tl.getMagnitude());
		// bottomRight.setVector(br.getMagnitude());		
		// bottomLeft.setVector(bl.getMagnitude());
	}

	public static Drivetrain getInstance() {
		if(instance == null) 
			instance = new Drivetrain();
		return instance;
	}


	public double max4(double a, double b, double c, double d) {
		return Math.max(Math.max(a, b), Math.max(c, d));
	}

	public void initRotationVectors()
	{
		Vector[] vecs = new Vector[4];

		for(int j = 0; j < vecs.length; j++)
		{
			switch(j)
			{
				case 0:
					vecs[j] = new Vector(DT_WIDTH, DT_LENGTH);
					break;
				case 1:
					vecs[j] = new Vector(DT_LENGTH, -DT_WIDTH);
					break;
				case 2:
					vecs[j] = new Vector(-DT_WIDTH, -DT_LENGTH);
					break;
				case 3:
					vecs[j] = new Vector(-DT_LENGTH, DT_WIDTH);
					break;
			}
		}


		for(Vector vec : vecs)
		{
			vec.scale(1.0/Math.max(DT_LENGTH, DT_WIDTH));
		}

		rotationVectors = vecs;
	}
}