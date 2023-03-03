package com.team8013.frc2023.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.team8013.frc2023.Constants;
import com.team8013.frc2023.Ports;
import com.team8013.frc2023.loops.ILooper;
import com.team8013.frc2023.loops.Loop;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDs extends Subsystem {

    private final CANdle mCandle = new CANdle(Ports.CANDLE, "canivore1");

    private static LEDs mInstance;

    public static LEDs getInstance() {
        if (mInstance == null) {
            mInstance = new LEDs();
        }
        return mInstance;
    }

    private double timestamp = 0.0;

    private final boolean mUseSmartdash = false; // if we want to manual control lights using shuffleboard

    // led sections
    // private LEDStatus mTopStatus = new LEDStatus(14, 28);
    private LEDStatus mStatus = new LEDStatus(0, 56);
    // private LEDStatus mFrontStatus = new LEDStatus(25, 56);

    // shuffleboard selectors
    // private SendableChooser<State> mFrontStateChooser;
    private SendableChooser<State> mStateChooser;

    // animation to run when disabled
    private Animation mRedAllianceAnimation = new ColorFlowAnimation(255, 0, 0, 0, 0.5, 56, Direction.Forward);
    private Animation mBlueAllianceAnimation = new ColorFlowAnimation(0, 0, 255, 0, 0.5, 56, Direction.Backward);
    private Animation mNoAllianceAnimation = new ColorFlowAnimation(185, 64, 255, 0, 0.5, 56, Direction.Backward);
    private Animation mChampsRainbowAutoAnimation = new RainbowAnimation(1.0, 1.0, 56);

    public ColorChoices mAllianceColor = ColorChoices.NONE;
    public ColorChoices mMatchedColor;

    public enum ColorChoices {
        RED, BLUE, OTHER, NONE
    }

    // led states
    public enum State {
        OFF("OFF", Double.POSITIVE_INFINITY, Color.off()),
        EMERGENCY("EMERGENCY", 0.05, new Color(255, 0, 0), Color.off()),

        SOLID_RED("SOLID_RED", Double.POSITIVE_INFINITY, new Color(255, 0, 0)),
        SOLID_PINK("SOLID_PINK", Double.POSITIVE_INFINITY, new Color(255, 18, 143)),
        SOLID_GREEN("SOLID_GREEN", Double.POSITIVE_INFINITY, new Color(0, 255, 8)),
        SOLID_PURPLE("SOLID_PURPLE", Double.POSITIVE_INFINITY, new Color(196, 18, 255)),
        SOLID_ORANGE("SOLID_ORANGE", Double.POSITIVE_INFINITY, new Color(255, 53, 13)),
        SOLID_YELLOW("SOLID_YELLOW", Double.POSITIVE_INFINITY, new Color(255, 150, 5)),
        SOLID_CYAN("SOLID_CYAN", Double.POSITIVE_INFINITY, new Color(18, 239, 255)),
        SOLID_BLUE("SOLID_BLUE", Double.POSITIVE_INFINITY, new Color(0, 0, 255)),

        FLASHING_RED("FLASHING_RED", 0.05, new Color(255, 0, 0), Color.off()),
        FLASHING_PINK("FLASHING_PINK", 0.05, new Color(255, 20, 0), Color.off()),
        FLASHING_GREEN("FLASHING_GREEN", 0.05, new Color(0, 255, 0), Color.off()),
        FLASHING_PURPLE("FLASHING_PURPLE", 0.05, new Color(255, 0, 255), Color.off()),
        FLASHING_ORANGE("FLASHING_ORANGE", 0.05, new Color(255, 53, 0), Color.off()),
        FLASHING_YELLOW("FLASHING_YELLOW", 0.05, new Color(255, 247, 5), Color.off()),
        FLASHING_CYAN("FLASHING_CYAN", 0.05, new Color(0, 255, 255), Color.off());

        Color[] colors; // array of colors to iterate over
        double interval; // time in seconds between states
        String name; // name of state

        private State(String name, double interval, Color... colors) {
            this.colors = colors;
            this.interval = interval;
            this.name = name;
        }

        public String getName() {
            return name;
        }
    }

    public LEDs() {
        configureCandle(); // set CTRE configurations for CANdle

        mMatchedColor = ColorChoices.NONE;

        // create sendable choosers for shuffleboard
        if (mUseSmartdash) {
            // mFrontStateChooser = new SendableChooser<>();
            mStateChooser = new SendableChooser<>();
            for (State state : State.values()) {
                // mFrontStateChooser.addOption(state.getName(), state);
                mStateChooser.addOption(state.getName(), state);
            }
            // mFrontStateChooser.setDefaultOption("OFF", State.OFF);
            mStateChooser.setDefaultOption("OFF", State.OFF);
            SmartDashboard.putData("LEDs", mStateChooser);
            // SmartDashboard.putData("Back LEDs", mBackStateChooser);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // empty
            }

            @Override
            public void onLoop(double timestamp) {
                // empty
            }

            @Override
            public void onStop(double timestamp) {
                // mFrontStatus.reset();
                mStatus.reset();
            }
        });
    }

    @Override
    public void readPeriodicInputs() {
        outputTelemtry();
        timestamp = Timer.getFPGATimestamp(); // update timestamp for color cycling
        if (mUseSmartdash) { // pull states from smartdash
            applyStates(mStateChooser.getSelected());
        }
    }

    public void updateState() { /// write state to candle
        // alternate updating each section of leds to avoid overrunning the candle
        updateLeds();
    }

    private void updateLeds() {
        // check if we need to cycle to next color
        if (mStatus.state.interval != Double.POSITIVE_INFINITY) {
            if (timestamp - mStatus.lastSwitchTime >= mStatus.state.interval) {
                mStatus.nextColor();
                mStatus.lastSwitchTime = timestamp;
            }
        }

        Color mColor = mStatus.getWantedColor();

        mCandle.setLEDs(
                mColor.r,
                mColor.g,
                mColor.b, 0, mStatus.startIDx,
                mStatus.LEDCount);
    }

    // setter functions
    public void applyStates(State state) {
        mStatus.setState(state);
        // mFrontStatus.setState(frontState);
    }

    // apply configuration to candle
    private void configureCandle() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1.0;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        mCandle.configAllSettings(configAll, Constants.kLongCANTimeoutMs);
        mCandle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 255);
        mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_1_General, 10);
        mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_2_ModulatedVBatOut, 255);
    }

    public void updateColor(ColorChoices allianceColor) {
        if (allianceColor.equals(ColorChoices.RED)) {
            mCandle.animate(mRedAllianceAnimation);
        } else if (allianceColor.equals(ColorChoices.BLUE)) {
            mCandle.animate(mBlueAllianceAnimation);
        } else {
            mCandle.animate(mNoAllianceAnimation);
        }
    }

    // update our alliance color
    // only should be updated in disabled periodic
    public void updateAllianceColor() {
        if (DriverStation.isDSAttached()) {
            if (edu.wpi.first.wpilibj.DriverStation.getAlliance() == Alliance.Red) {
                mAllianceColor = ColorChoices.RED;
            } else if (edu.wpi.first.wpilibj.DriverStation.getAlliance() == Alliance.Blue) {
                mAllianceColor = ColorChoices.BLUE;
            }
        } else {
            mAllianceColor = ColorChoices.NONE;
            DriverStation.reportError("No Alliance Color Detected", true);
        }
    }

    public ColorChoices getAllianceColor() {
        return mAllianceColor;
    }

    public void setChampsAutoAnimation() {
        mCandle.animate(mChampsRainbowAutoAnimation, 0);
    }

    public void clearAnimation() {
        mCandle.clearAnimation(0);
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    // getter functions
    public State getState() {
        return mStatus.state;
    }

    // public State getBackState() {
    // return mBackStatus.state;
    // }

    public boolean getUsingSmartdash() {
        return mUseSmartdash;
    }

    private void outputTelemtry() {
        SmartDashboard.putString("LED Status", getState().name);
        // SmartDashboard.putString("Back LED Status", getBackState().name);

        SmartDashboard.putString("LED Colors", mStatus.getWantedColor().toString());
        // SmartDashboard.putString("Back LED Colors",
        // mBackStatus.getWantedColor().toString());
    }

    // class for holding information about each section
    private class LEDStatus {
        private State state = State.OFF; // current state
        private double lastSwitchTime = 0.0; // timestampe of last color cycle
        private int colorIndex = 0; // tracks current color in array
        private int startIDx, LEDCount; // start and end of section

        public LEDStatus(int startIndex, int endIndex) {
            startIDx = startIndex;
            LEDCount = endIndex - startIndex;
        }

        public void setState(State wantedState) {
            if (wantedState != state) {
                colorIndex = 0;
                lastSwitchTime = Timer.getFPGATimestamp();
                state = wantedState;
            }
        }

        public Color getWantedColor() {
            Color color;
            try {
                color = state.colors[colorIndex];
            } catch (Exception e) {
                color = Color.off();
            }
            return color;
        }

        // cycle to next color in array
        public void nextColor() {
            if (state.colors.length == 1) {
                return;
            }
            if (colorIndex == state.colors.length - 1) {
                colorIndex = 0;
            } else {
                colorIndex++;
            }
        }

        public void reset() {
            state = State.OFF;
            lastSwitchTime = 0.0;
            colorIndex = 0;
        }
    }

    // class to hold rgb values for a color
    private static class Color {
        public int r;
        public int g;
        public int b;

        public Color(int red, int green, int blue) {
            r = red;
            g = green;
            b = blue;
        }

        public static Color off() {
            return new Color(0, 0, 0);
        }

        @Override
        public String toString() {
            return "(" + r + "," + g + "," + b + ")";
        }
    }
}
