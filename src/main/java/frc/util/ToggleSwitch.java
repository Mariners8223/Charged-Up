// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util;

import com.google.flatbuffers.Constants;

import frc.robot.subsystems.FaultChecker;

/** Add your docs here. */
public class ToggleSwitch {
    
    private Boolean[] toggleLocation;

    public ToggleSwitch(){
        toggleLocation = new Boolean[frc.robot.Constants.ToggleSwitchCount];
        for(int i = 0; i < toggleLocation.length; i++){
            toggleLocation[i] = false;
        }
    }

    //toggle speed is number 0
    public void toggleTheSwitch(int number){
        toggleLocation[number] = !toggleLocation[number];
    }

    public boolean getToggleSwitchValue(int number){
        return toggleLocation[number];
    }

    public boolean ToggleTheSwitchAndGetToggleSwitchValue(int number){
        toggleLocation[number] = !toggleLocation[number];
        return toggleLocation[number];
    }
}

