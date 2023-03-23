package com.team8013.frc2023.shuffleboard;

import java.util.ArrayList;
import java.util.List;

import com.team8013.frc2023.shuffleboard.tabs.SuperstructureTab;
import com.team8013.frc2023.shuffleboard.tabs.SwerveTab;
import com.team8013.frc2023.shuffleboard.tabs.ArmTab;
import com.team8013.frc2023.shuffleboard.tabs.ClawTab;
import com.team8013.frc2023.shuffleboard.tabs.LedTab;
import com.team8013.frc2023.shuffleboard.tabs.PivotTab;

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

    //private OperatorTab mOperatorTab;
    private FieldView mFieldView = new FieldView();

    // instantiate subsystems, tabs, and widgets
    public ShuffleBoardInteractions() {

    List<ShuffleboardTabBase> optionalTabs = List.of(
            new SwerveTab(),
            new SuperstructureTab(),
            new ClawTab(),
            new ArmTab(),
            new PivotTab(),
            new LedTab()
            );

        mTabs.addAll(optionalTabs);

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

}
 