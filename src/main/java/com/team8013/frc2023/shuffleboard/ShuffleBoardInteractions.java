package com.team8013.frc2023.shuffleboard;

import java.util.ArrayList;
import java.util.List;

import com.team8013.frc2023.shuffleboard.tabs.SuperstructureTab;
import com.team8013.frc2023.shuffleboard.tabs.SwerveTab;
import com.team8013.frc2023.shuffleboard.tabs.SystemsTab;
import com.team8013.frc2023.shuffleboard.tabs.VisionTab;
import com.team8013.frc2023.shuffleboard.tabs.OperatorTab;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleBoardInteractions {

    // Trims unneccesary tabs when in competition
    public final boolean mDebug = false;

    /* ShuffleBoardInteractions Instance */
    private static ShuffleBoardInteractions mInstance; 

    public static ShuffleBoardInteractions getInstance() {
        if (mInstance == null) {
            mInstance = new ShuffleBoardInteractions();
        }
        return mInstance;
    }

    private ArrayList<ShuffleboardTabBase> mTabs = new ArrayList<ShuffleboardTabBase>();

    private OperatorTab mOperatorTab;
    private FieldView mFieldView = new FieldView();

    // instantiate subsystems, tabs, and widgets
    public ShuffleBoardInteractions() {
        mOperatorTab = new OperatorTab();
        // mTabs.add(mOperatorTab);

        if (mDebug) {
            List<ShuffleboardTabBase> optionalTabs = List.of(
                new SwerveTab(),
                new SuperstructureTab(),
                new VisionTab()
                // new ColorSensorTab(),
                // new ShooterTab(),
                // new ClimberTab(),
                // new IndexerTab(),
                // new IntakeTab(),
                // new LedTab(),
                // new ManualShooterTab()
            );
            mTabs.addAll(optionalTabs);
        } else {
            mTabs.add(new SystemsTab());
        }

        for(ShuffleboardTabBase tab: mTabs) {
            tab.createEntries();
        }
    }

    public void update() {
        for (ShuffleboardTabBase tab : mTabs) {
            tab.update();
        }
        mFieldView.update();
    }

    public ShuffleboardTab getOperatorTab() {
         return mOperatorTab.getTab();
    }
}
 