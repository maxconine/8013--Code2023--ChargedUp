// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team8013.frc2023;

import java.util.Optional;

import com.lib.util.CTREConfigs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.Timer;
import com.team254.lib.wpilib.TimedRobot;
import com.team8013.frc2023.auto.AutoModeExecutor;
import com.team8013.frc2023.auto.AutoModeSelector;
import com.team8013.frc2023.auto.modes.AutoModeBase;
import com.team8013.frc2023.controlboard.ControlBoard;
import com.team8013.frc2023.controlboard.ControlBoard.SwerveCardinal;
import com.team8013.frc2023.logger.LoggingSystem;
import com.team8013.frc2023.loops.CrashTracker;
import com.team8013.frc2023.loops.Looper;
import com.team8013.frc2023.shuffleboard.ShuffleBoardInteractions;
import com.team8013.frc2023.subsystems.Limelight;
import com.team8013.frc2023.subsystems.RobotStateEstimator;
import com.team8013.frc2023.subsystems.Superstructure;
import com.team8013.frc2023.subsystems.Swerve;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any
	 * initialization code.
	 */

	 
	// instantiate enabled and disabled loopers
	private final Looper mEnabledLooper = new Looper();
	private final Looper mDisabledLooper = new Looper();
	// instantiate logging looper
	private final Looper mLoggingLooper = new Looper();

	// declare necessary class objects
	private ShuffleBoardInteractions mShuffleBoardInteractions;
	public static CTREConfigs ctreConfigs;

	// subsystem instances
	private final ControlBoard mControlBoard = ControlBoard.getInstance();

	private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
	private final Superstructure mSuperstructure = Superstructure.getInstance();
	private final Swerve mSwerve = Swerve.getInstance();
	// private final Intake mIntake = Intake.getInstance();
	// private final Indexer mIndexer = Indexer.getInstance();
	// private final Shooter mShooter = Shooter.getInstance();
	// private final Trigger mTrigger = Trigger.getInstance();
	// private final Hood mHood = Hood.getInstance();
	// private final ColorSensor mColorSensor = ColorSensor.getInstance();
	// private final Climber mClimber = Climber.getInstance();
	private final Limelight mLimelight = Limelight.getInstance();
	// private final LEDs mLEDs = LEDs.getInstance();

	// robot state estimator
	private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();

	// logging system
	private LoggingSystem mLogger = LoggingSystem.getInstance();

	// auto instances
	private AutoModeExecutor mAutoModeExecutor;
	private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

	public Robot() {
		CrashTracker.logRobotConstruction();
	}

	@Override
	public void robotInit() {

		ctreConfigs = new CTREConfigs();
		mShuffleBoardInteractions = ShuffleBoardInteractions.getInstance();

		try {
			CrashTracker.logRobotInit();

			mSubsystemManager.setSubsystems(			
					mRobotStateEstimator,
					mSwerve,
					mSuperstructure,
					// mIntake,
					// mIndexer,
					// mShooter,
					// mTrigger,
					// mHood,
					// mColorSensor,
					// mClimber,
					mLimelight
					//mLEDs
			);

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);
			
			mSubsystemManager.registerLoggingSystems(mLogger);
            mLogger.registerLoops(mLoggingLooper);

			RobotState.getInstance().reset(Timer.getFPGATimestamp(), new com.team254.lib.geometry.Pose2d());
			mSwerve.resetOdometry(new Pose2d());
			mSwerve.resetAnglesToAbsolute();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void robotPeriodic() {
		mEnabledLooper.outputToSmartDashboard();
		mShuffleBoardInteractions.update();
		// mClimber.outputTelemetry();
	}

	@Override
	public void autonomousInit() {
		CrashTracker.logAutoInit();

		try {
			// reset states
			mSuperstructure.stop();

			mDisabledLooper.stop();
			mEnabledLooper.start();
			mLoggingLooper.start();

			Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
			if (autoMode.isPresent()) {
				mSwerve.resetOdometry(autoMode.get().getStartingPose());
			}

			mAutoModeExecutor.start();

			mLimelight.setPipeline(Constants.VisionConstants.kDefaultPipeline);

			// set champs pride automation
			// mLEDs.setChampsAutoAnimation();	

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

	}

	@Override
	public void autonomousPeriodic() {
		mLimelight.setLed(Limelight.LedMode.ON);
		// mLEDs.updateState();
		// mLEDs.applyStates(State.SOLID_BLUE, State.SOLID_YELLOW);
	}

	@Override
	public void teleopInit() {
		try {

			if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

			mDisabledLooper.stop();
			mEnabledLooper.start();
			mLoggingLooper.start();

			// mSuperstructure.setWantEject(false, false);

			// mClimber.setBrakeMode(true);

			// mSuperstructure.setEjectDisable(false);

			mLimelight.setLed(Limelight.LedMode.ON);
            mLimelight.setPipeline(Constants.VisionConstants.kDefaultPipeline);

			// clear any previous automation from auto
			// mLEDs.clearAnimation();

			// set states for teleop init
			// mSuperstructure.setInitialTeleopStates();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopPeriodic() {
		try {

			if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

			mLimelight.setLed(Limelight.LedMode.ON);
			mLimelight.outputTelemetry();

			// call operator commands container from superstructure
			// mSuperstructure.updateOperatorCommands();
			// mSuperstructure.updateLEDs();

			// mLEDs.updateState();

			
			/* SWERVE DRIVE */
			// far left switch up
			if (mControlBoard.getBrake()) {
				mSwerve.setLocked(true);
			} else {
				mSwerve.setLocked(false);
			}

			if (mControlBoard.zeroGyro()) {
				mSwerve.zeroGyro();
			}

			if (mControlBoard.getSwerveSnap() != SwerveCardinal.NONE) {
				mSwerve.startSnap(mControlBoard.getSwerveSnap().degrees);
			}

			Translation2d swerveTranslation = new Translation2d(mControlBoard.getSwerveTranslation().x(),
					mControlBoard.getSwerveTranslation().y());
			double swerveRotation = mControlBoard.getSwerveRotation();

			if (mControlBoard.getVisionAlign()) {
				mSwerve.visionAlignDrive(swerveTranslation, true);
			} else {
				mSwerve.drive(swerveTranslation, swerveRotation, true, true);
			}

		} catch (Throwable t) {
			t.printStackTrace();
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledInit() {
		try {
			// reset states
			mSuperstructure.stop();

			CrashTracker.logDisabledInit();
			mEnabledLooper.stop();
			mDisabledLooper.start();

			mLoggingLooper.stop();

			mLimelight.setLed(Limelight.LedMode.ON);
            mLimelight.triggerOutputs();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

		if (mAutoModeExecutor != null) {
			mAutoModeExecutor.stop();
		}

		// Reset all auto mode state.
		mAutoModeSelector.reset();
		mAutoModeSelector.updateModeCreator();
		mAutoModeExecutor = new AutoModeExecutor();

	}

	@Override
	public void disabledPeriodic() {
		try {

			mDisabledLooper.outputToSmartDashboard();

			mAutoModeSelector.updateModeCreator();
			
			mSwerve.resetAnglesToAbsolute();

			// update alliance color from driver station while disabled
			// mColorSensor.updateAllianceColor();
			// update linear offset for rb ratio
			//mColorSensor.updateColorOffset();

			//mLEDs.updateColor(mColorSensor.getAllianceColor());

			mLimelight.setLed(Limelight.LedMode.ON);
			mLimelight.writePeriodicOutputs();
			mLimelight.outputTelemetry();

			Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
			if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
				System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
				mAutoModeExecutor.setAutoMode(autoMode.get());
			}

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {
		try {
			mDisabledLooper.stop();
			mEnabledLooper.stop();

			mLoggingLooper.stop();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testPeriodic() {
	}
}
