// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team8013.frc2023;

import java.util.Optional;

import com.lib.util.CTREConfigs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team254.lib.wpilib.TimedRobot;
import com.team8013.frc2023.auto.AutoModeExecutor;
import com.team8013.frc2023.auto.AutoModeSelector;
import com.team8013.frc2023.auto.modes.AutoModeBase;
import com.team8013.frc2023.controlboard.ControlBoard;
import com.team8013.frc2023.drivers.Pigeon;
import com.team8013.frc2023.logger.LoggingSystem;
import com.team8013.frc2023.loops.CrashTracker;
import com.team8013.frc2023.loops.Looper;
import com.team8013.frc2023.shuffleboard.ShuffleBoardInteractions;
import com.team8013.frc2023.subsystems.Arm;
import com.team8013.frc2023.subsystems.ClawV2;
import com.team8013.frc2023.subsystems.Limelight;
//import com.team8013.frc2023.subsystems.Pivot;
import com.team8013.frc2023.subsystems.PivotV2;
import com.team8013.frc2023.subsystems.LEDs;
import com.team8013.frc2023.subsystems.LEDs.State;
import com.team8013.frc2023.subsystems.Limelight.LedMode;
import com.team8013.frc2023.subsystems.RobotStateEstimator;
import com.team8013.frc2023.subsystems.Superstructure;
import com.team8013.frc2023.subsystems.Swerve;
import edu.wpi.first.cameraserver.CameraServer;

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
	// hitest
	// instantiate enabled and disabled loopers
	private final Looper mEnabledLooper = new Looper();
	private final Looper mDisabledLooper = new Looper();
	// instantiate logging looper
	private final Looper mLoggingLooper = new Looper();
	public static int maxArmPosition;
	public static boolean settingDown, settingPickup, settingHybrid, settingMid, settingHigh;
	public static boolean canControlArmManually;
	// declare necessary class objects
	private ShuffleBoardInteractions mShuffleBoardInteractions;
	public static CTREConfigs ctreConfigs;

	// subsystem instances
	private final ControlBoard mControlBoard = ControlBoard.getInstance();

	private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
	private final Superstructure mSuperstructure = Superstructure.getInstance();
	private final Swerve mSwerve = Swerve.getInstance();
	// private final Intake mIntake = Intake.getInstance();
	private final PivotV2 mPivot = PivotV2.getInstance();
	// private final Shooter mShooter = Shooter.getInstance();
	// private final Trigger mTrigger = Trigger.getInstance();
	// private final Hood mHood = Hood.getInstance();
	private final ClawV2 mClaw = ClawV2.getInstance();
	private final Arm mArm = Arm.getInstance();
	private final Limelight mLimelight = Limelight.getInstance();
	private final LEDs mLEDs = LEDs.getInstance();
	private final Pigeon mPigeon = Pigeon.getInstance();

	// robot state estimator
	private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();

	// logging system
	private LoggingSystem mLogger = LoggingSystem.getInstance();

	// auto instances
	private AutoModeExecutor mAutoModeExecutor;
	private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

	public boolean[] autoBalance;
	public Timer autoTimer = new Timer();

	public Robot() {
		CrashTracker.logRobotConstruction();
		System.out.println("robots construction");
	}

	@Override
	public void robotInit() {

		mPigeon.zero();

		ctreConfigs = new CTREConfigs();
		mShuffleBoardInteractions = ShuffleBoardInteractions.getInstance();
		mLimelight.setPipeline(3);

		CameraServer.startAutomaticCapture();
		CameraServer.startAutomaticCapture();

		try {
			CrashTracker.logRobotInit();

			mSubsystemManager.setSubsystems(
					mRobotStateEstimator,
					mSwerve,
					mSuperstructure,
					mPivot,
					mClaw,
					mArm,
					mLimelight,
					mLEDs);

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			mSubsystemManager.registerLoggingSystems(mLogger);
			mLogger.registerLoops(mLoggingLooper);

			RobotState.getInstance().reset(Timer.getFPGATimestamp(), new com.team254.lib.geometry.Pose2d());
			mSwerve.resetOdometry(new Pose2d());
			mSwerve.resetAnglesToAbsolute();

			mLEDs.applyStates(State.SOLID_BLUE);

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void robotPeriodic() {
		mEnabledLooper.outputToSmartDashboard();
		mShuffleBoardInteractions.update();
		mClaw.outputTelemetry();
		mArm.outputTelemetry();
		mPivot.outputTelemetry();
	}

	@Override
	public void autonomousInit() {
		CrashTracker.logAutoInit();

		try {

			// reset states
			mSuperstructure.stop();

			mArm.pullArmIntoZero();

			mDisabledLooper.stop();
			mEnabledLooper.start();
			mLoggingLooper.start();
			mLimelight.setLed(LedMode.PIPELINE); // Set Limelight LED's to Pipeline settings (shield your eyes!)

			Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
			if (autoMode.isPresent()) {
				mSwerve.resetOdometry(autoMode.get().getStartingPose());
			}

			System.out.println("Before starting auto mode executor");

			mAutoModeExecutor.start();

			System.out.println("After starting auto mode executor");

			// TODO:
			// mLimelight.setPipeline(Constants.VisionConstants.kDefaultPipeline);

			// set champs pride automation
			mLEDs.setChampsAutoAnimation();

			mSuperstructure.zeroRollInit();

		} catch (Throwable t) {
			System.out.println("crash tracker for auto");
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

	}

	@Override
	public void autonomousPeriodic() {

		SmartDashboard.putNumber("PIGEON Pitch", mPigeon.getPitch().getDegrees());
		SmartDashboard.putNumber("PIGEON roll", mPigeon.getRoll().getDegrees());
		// TODO;
		mLimelight.setLed(Limelight.LedMode.ON);
		mLEDs.updateState();

		// stops the claw from opening too far, makes auto more efficient
		mSuperstructure.autoPeriodic();

		mSuperstructure.autoBalanceNonPID();

		// autoBalance = mSuperstructure.getAutoBalance(); //2d array: want auto
		// balance, fromBack

		// if (autoBalance[0]) {
		// if (mPigeon.getRoll().getDegrees() > 3) {
		// if (autoBalance[1]) {
		// mSwerve.drive(new Translation2d(.4, 0), 0, true, false);
		// } else {
		// mSwerve.drive(new Translation2d(.6, 0), 0, true, false);
		// }
		// SmartDashboard.putBoolean("ChargeStation", false);
		// SmartDashboard.putBoolean("going forwards balance", false);
		// } else if (mPigeon.getRoll().getDegrees() < -3) {
		// if (autoBalance[1]) {
		// mSwerve.drive(new Translation2d(-.6, 0), 0, true, false);
		// } else {
		// mSwerve.drive(new Translation2d(-.3, 0), 0, true, false);
		// }
		// SmartDashboard.putBoolean("ChargeStation", false);
		// SmartDashboard.putBoolean("going forwards balance", false);
		// } else {
		// autoTimer.start();
		// if (autoTimer.get() > 1) {
		// mSwerve.setLocked(true);
		// SmartDashboard.putBoolean("ChargeStation", true);
		// }
		// }
		// }

	}

	@Override
	public void teleopInit() {
		try {
			// mSwerve.zeroGyro();

			mLimelight.setPipeline(3);

			if (mAutoModeExecutor != null) {
				mAutoModeExecutor.stop();
			}

			mClaw.stopPivot();
			mClaw.setPivotTeleopInit();

			// mClaw.setPivotPosition(0);
			mArm.pullArmIntoZero(); // TODO: DO WE NEED THIS???

			mDisabledLooper.stop();
			mEnabledLooper.start();
			mLoggingLooper.start();

			// mPivot.setPivotDown(); //TODO: CHANGED THiS IN CASE WE DON'T END AUTO DOWN IT
			// DOESNT BRAKE THE ARM

			mLimelight.setLed(LedMode.PIPELINE); // Set Limelight LED's to Pipeline settings (shield your eyes!)

			// TODO:
			// mLimelight.setLed(Limelight.LedMode.ON);
			// mLimelight.setPipeline(Constants.VisionConstants.kDefaultPipeline);

			// clear any previous automation from auto
			mLEDs.clearAnimation();
			mLEDs.applyStates(State.OFF);

			// set states for teleop init
			mSuperstructure.setInitialTeleopStates();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopPeriodic() {
		try {
			// mSuperstructure.readPeriodicInputs();

			// SmartDashboard.putNumber("PIGEON Pitch", mPigeon.getPitch().getDegrees());
			// SmartDashboard.putNumber("PIGEON roll", mPigeon.getRoll().getDegrees());

			if (mAutoModeExecutor != null) {
				mAutoModeExecutor.stop();
			}

			// TODO:
			// mLimelight.setLed(Limelight.LedMode.ON);
			// mLimelight.outputTelemetry();

			// call operator commands container from superstructure
			mSuperstructure.updateOperatorCommands();
			mSuperstructure.updateLEDs();

			mLEDs.updateState();

			// mSwerve.getBotPose();

			/* SWERVE DRIVE */
			// far left switch up
			if (mControlBoard.getBrake()) {
				mSwerve.setLocked(true);
				SmartDashboard.putBoolean("Brake On", true);
			} else {
				mSwerve.setLocked(false);
				SmartDashboard.putBoolean("Brake On", false);
			}

			if (mControlBoard.zeroGyro()) {
				mSwerve.zeroGyro();
			}

			// if (mControlBoard.getSwerveSnap() != SwerveCardinal.NONE) {
			// mSwerve.startSnap(mControlBoard.getSwerveSnap().degrees);
			// SmartDashboard.putNumber("Snapping Drgrees",
			// mControlBoard.getSwerveSnap().degrees);
			// } else {
			// SmartDashboard.putNumber("Snapping Drgrees", -1);
			// }

			Translation2d swerveTranslation = new Translation2d(mControlBoard.getSwerveTranslation().x(),
					mControlBoard.getSwerveTranslation().y());
			double swerveRotation = mControlBoard.getSwerveRotation();

			// if (mControlBoard.getVisionAlign()) {
			// mSwerve.visionAlignDrive(swerveTranslation, true);
			// SmartDashboard.putBoolean("trying to vision allign drive", true);
			// } else {
			mSwerve.drive(swerveTranslation, swerveRotation, true, true);
			SmartDashboard.putBoolean("trying to vision allign drive", false);
			// }

			/* Smart Dashboard outputs */
			SmartDashboard.putNumber("FPGAT Timestamp", Timer.getFPGATimestamp());

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

			mArm.setBrakeMode(true);

			if (mControlBoard.getWantCone()) {
				mPivot.setBrakeMode(true);
			} else {
				mPivot.setBrakeMode(false);
			}

			// TODO:
			// mLimelight.setLed(Limelight.LedMode.ON);
			// mLimelight.triggerOutputs();

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
			mLimelight.setLed(LedMode.OFF); // Turn off Limelight LED's if robot is disabled (your eyes will thank me)

			// update alliance color from driver station while disabled
			mLEDs.updateAllianceColor();

			mLEDs.updateColor(mLEDs.getAllianceColor());

			mLimelight.writePeriodicOutputs();
			// mLimelight.outputTelemetry();
			mClaw.outputTelemetry();

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
