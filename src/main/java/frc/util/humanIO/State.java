// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.humanIO;

/** 
 * Enum used in the JoystickAxis with the getCurrentState() method.
*/
public enum State {
    letGo(0),
    held(1),
    fullyHeld(2);

    public final int value;

    State(int value) {
      this.value = value;
    }
  }