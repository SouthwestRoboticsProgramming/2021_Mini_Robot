package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Utils;
import frc.lib.PID;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveTrainSubsystem extends SubsystemBase {

	private final WPI_TalonSRX leftMaster, rightMaster;
	private final double maxVelocity = 20000;
	private final double maxSpeed = 1;
	private Constants constants = new Constants();
	private Pose2d position;
	private DifferentialDriveOdometry differentialDriveOdometry;
	private TalonSRXConfiguration srxConfig;
	private boolean shiftyStarted = false;
	
	public enum WheelSide {
		left, right, both
	  }

	public DriveTrainSubsystem() {
	//setup motors
		leftMaster = new WPI_TalonSRX(Constants.leftPort1);
		rightMaster = new WPI_TalonSRX(Constants.rightPort1);

	// RESET TALONS
		leftMaster.configFactoryDefault();
		rightMaster.configFactoryDefault();

		// // PID
		srxConfig = new TalonSRXConfiguration();
		srxConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		srxConfig.neutralDeadband = .001;
		srxConfig.slot0.kF = Robot.shuffleBoard.driveFXPidF.getDouble(0);
		srxConfig.slot0.kP = Robot.shuffleBoard.driveFXPidP.getDouble(0);
		srxConfig.slot0.kI = Robot.shuffleBoard.driveFXPidI.getDouble(0);
		srxConfig.slot0.kD = Robot.shuffleBoard.driveFXPidD.getDouble(0);
		srxConfig.slot0.closedLoopPeakOutput = 1;
		srxConfig.openloopRamp = .5;


		rightMaster.configAllSettings(srxConfig);
		leftMaster.configAllSettings(srxConfig);
		
		// LEFT MASTER
		leftMaster.setInverted(false);
		leftMaster.setSensorPhase(true);
		leftMaster.setNeutralMode(NeutralMode.Brake);
		leftMaster.setSelectedSensorPosition(0, 0, 30);
		
		// RIGHT MASTER
		rightMaster.config_kP(0, 0);
		rightMaster.config_kI(0, 0);
		rightMaster.config_kD(0, 0);
		rightMaster.config_kF(0, 0);
		rightMaster.configClosedloopRamp(.5);
		rightMaster.setInverted(true);
		rightMaster.setSensorPhase(true);
		rightMaster.setSelectedSensorPosition(0, 0, 30);	
		rightMaster.setNeutralMode(NeutralMode.Brake);
		
		position = new Pose2d(0,0,getRHeading());
		differentialDriveOdometry = new DifferentialDriveOdometry(getRHeading());
		differentialDriveOdometry.resetPosition(position, getRHeading());
	}

	public TalonSRXConfiguration getTalonSRXConfig() {
		return srxConfig;
	}

	public void setTalonSRXConfig(TalonSRXConfiguration config) {
		srxConfig = config;
		rightMaster.configAllSettings(srxConfig);
		leftMaster.configAllSettings(srxConfig);
	}

	public void setFront(boolean reverse) {
		rightMaster.setInverted(reverse);
		leftMaster.setInverted(!reverse);
	}

	// GET INFO
	public Rotation2d getRHeading() {
		return Rotation2d.fromDegrees(getHeading());
	}
	public double getHeading() {
		// return 0;
		return Robot.gyro.getGyroAngleZ();
	}

	public WPI_TalonSRX getLeftMaster() {
		return leftMaster;
	}

	public WPI_TalonSRX getRightMaster() {
		return rightMaster;
	}

	public double getLeftOutput() {
		return leftMaster.get();
	}

	public double getRightOutput() {
		return rightMaster.get();
	}

	//GET ENCODOR OUTPUT
	public int getLeftDriveEncoderTicks() {
		return leftMaster.getSelectedSensorPosition();
	}

	public int getRightDriveEncoderTicks() {
		return rightMaster.getSelectedSensorPosition();
	}

	public double getLeftDriveFeet() {
		return leftMaster.getSelectedSensorPosition() / 4096 * Math.PI * 6 / 50;
	}

	public double getRightDriveFeet() {
		return rightMaster.getSelectedSensorPosition() / 4096 * Math.PI * 6 / 50;
	}

	// SET BRAKE MODE OF MOTORS
	public void setBrakeMode(boolean mode) {
		NeutralMode neutralMode;
		if (mode) {
			neutralMode = NeutralMode.Brake;
		} else {
			neutralMode = NeutralMode.Coast;
		}
		leftMaster.setNeutralMode(neutralMode);
		rightMaster.setNeutralMode(neutralMode);
	}

	public double getLeftVelocity() {
		return leftMaster.getSelectedSensorVelocity(0);
	}

	public double getLeftVelocityPercent() {
		return leftMaster.getSelectedSensorVelocity(0) / maxVelocity;
	}

	public double getRightVelocity() {
		return rightMaster.getSelectedSensorVelocity(0);
	}

	public double getRightVelocityPercent() {
		return rightMaster.getSelectedSensorVelocity(0) / maxVelocity;
	}

	public double percentToVelocity(double percent) {
		return (percent * maxVelocity)*2;
	}

	public void driveMotors(double left, double right) {
		driveMotors(left, right, ControlMode.PercentOutput);
		Robot.shuffleBoard.driveLeftOutput.setDouble(leftMaster.getMotorOutputPercent());
		Robot.shuffleBoard.driveRightOutput.setDouble(rightMaster.getMotorOutputPercent());
	}	

	public void stop() {
	leftMaster.stopMotor();
	rightMaster.stopMotor();
	}

	private double ticksToMeters(int ticks) {
		return(constants.wheelCircumferenceMeters / constants.encoderTicksPerRotation) * ticks;
	}

  @Override
  public void periodic() {
	  differentialDriveOdometry.update(getRHeading(), ticksToMeters(getLeftDriveEncoderTicks()) , ticksToMeters(getRightDriveEncoderTicks()));
	  Robot.shuffleBoard.drivePosition.setString(differentialDriveOdometry.getPoseMeters().toString());
  }

public double[] getTurnPid() {
	double[] turnPID = new double[] {Robot.shuffleBoard.driveTurnPidP.getDouble(0),
									 Robot.shuffleBoard.driveTurnPidI.getDouble(0),
									 Robot.shuffleBoard.driveTurnPidD.getDouble(0)};
	return turnPID;
}

public void driveMotors(double left, double right, ControlMode controlMode) {
	Utils utils = new Utils();
	left = utils.setRange(left, -maxSpeed, maxSpeed);
	right = utils.setRange(right, -maxSpeed, maxSpeed);
	leftMaster.set(controlMode, left);
	rightMaster.set(controlMode, right);
			Robot.shuffleBoard.driveLeftOutput.setDouble(leftMaster.getMotorOutputPercent());
			Robot.shuffleBoard.driveRightOutput.setDouble(rightMaster.getMotorOutputPercent());
}

public void driveAtAngle(double output, double angle, ControlMode controlMode) {
	double pidOutput = 0;
	// if (angle != Double.NaN) {
		// System.out.println("angle = " + angle);
		PID pid = new PID(getTurnPid()[0], getTurnPid()[1], getTurnPid()[2]);
		pid.setSetPoint(angle);
		pid.setActual(Robot.getAngle());
		pid.setError(getError(Robot.getAngle(), angle));
		pidOutput = pid.getOutput();
	// }
	
	// System.out.println("angleOffset = " + pid.getOutput() + " error = " + pid.getError());
	Utils utils = new Utils();
	output = utils.setRange(output, -maxSpeed, maxSpeed);
	double leftOutput = -output - pidOutput;
	double rightOutput = -output + pidOutput;
	leftMaster.set(controlMode, leftOutput);
	rightMaster.set(controlMode, rightOutput);
			Robot.shuffleBoard.driveLeftOutput.setDouble(leftMaster.getMotorOutputPercent());
			Robot.shuffleBoard.driveRightOutput.setDouble(rightMaster.getMotorOutputPercent());
}

public static double getError(double act, double set) {
	if (act >= 0) {
		double a = -(-set+act);
		double b = 360-(act - set);
		if (a <= 180 && a >= -180) {
			return a;
		} else {
			return b;
		}
	} else {
		double c = -(-set+act);
		double d = -(360-(-act + set));
		if (c <= 180 && c >= -180) {
			return c;
		} else {
			return d;
		}
	}		
}

}
