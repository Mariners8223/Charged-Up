// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util;

import com.google.flatbuffers.Constants;

import frc.robot.subsystems.FaultChecker;

/** Add your docs here. */
public class ToggleSwitch {
    
    private static int[] toggleLocation;

    public ToggleSwitch(){
        toggleLocation = new int[frc.robot.Constants.ToggleSwitchCount];
        for(int i = 0; i < toggleLocation.length; i++){
            toggleLocation[i] = 0;
        }
    }

    //toggle speed is number 0
    public static void toggleTheSwitch(int number){
        switch(toggleLocation[number]){
            default:
                toggleLocation[number] = 0;
                break;

            case 0:
                toggleLocation[number] = 1;
                break;

            case 1:
                toggleLocation[number] = 0;
                break;
        }
    }

    public static boolean getToggleSwitchValue(int number){
        switch(toggleLocation[number]){
            case 0:
                return false;

            case 1:
                return true;
            
            default:
                return false;
        }
    }

    public static boolean ToggleTheSwitchAndgetToggleSwitchValue(int number){
        switch(toggleLocation[number]){
            default:
                toggleLocation[number] = 0;
                return false;

            case 0:
                toggleLocation[number] = 1;
                return false;

            case 1:
                toggleLocation[number] = 0;
                return true;
        }
    }
}

