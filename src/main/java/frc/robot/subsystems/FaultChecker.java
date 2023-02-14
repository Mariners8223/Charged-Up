// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FaultChecker extends SubsystemBase {
    private static FaultChecker instance; 
    // Could potentionally make this
    private int curIndex = 0;
    private ArrayList<String> motorIds;
    private ArrayList<BaseMotorController> motors; 
    /** Creates a new FaultChecker. */
    private FaultChecker() {
        motorIds = new ArrayList<String>();
        motors = new ArrayList<BaseMotorController>();
    }

    public static FaultChecker getInstance()
    {
        if (instance == null)
            instance = new FaultChecker();
        
        return instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        for (int i = 0; i < Constants.MOTORS_CHECKED_PER_TICK; i++)
        {
            checkMotor(motorIds.get((curIndex + i) % motors.size()), motors.get((curIndex + i) % motors.size()));
        }
        curIndex = (curIndex + Constants.MOTORS_CHECKED_PER_TICK) % motors.size();
    }

    public void addMotor(String id, BaseMotorController motor)
    {
        motorIds.add(id);
        motors.add(motor);
    }

    private void checkMotor(String id, BaseMotorController motor)
    {
        Faults f = new Faults();
        ErrorCode errorCode = motor.getFaults(f);

        Logger.getInstance().recordOutput("lastErrorCode" + id, errorCode.value);
        if (f.hasAnyFault() || errorCode.value != 0)
        {
            new PrintCommand("MOTOR " + id + " HAS PROBLEMS!\n");
            if (f.hasAnyFault()) {

                /// TODO: add actual descriptions for every fault. See javadocs for each fault.
                StringBuilder work = new StringBuilder();

                if (f.UnderVoltage) work.append("UnderVoltage ");
                if (f.ForwardLimitSwitch) work.append( "ForwardLimitSwitch ");
                if (f.ReverseLimitSwitch) work.append( "ReverseLimitSwitch ");
                if (f.ForwardSoftLimit) work.append( "ForwardSoftLimit ");
                if (f.ReverseSoftLimit) work.append( "ReverseSoftLimit ");
                if (f.HardwareFailure) work.append( "HardwareFailure ");
                if (f.ResetDuringEn) work.append( "ResetDuringEn ");
                if (f.SensorOverflow) work.append( "SensorOverflow ");
                if (f.SensorOutOfPhase) work.append( "SensorOutOfPhase ");
                if (f.HardwareESDReset) work.append( "HardwareESDReset ");
                if (f.RemoteLossOfSignal) work.append( "RemoteLossOfSignal ");
                if (f.APIError) work.append( "APIError ");
                if (f.SupplyOverV) work.append( "SupplyOverV ");
                if (f.SupplyUnstable) work.append( "SupplyUnstable ");
                String result = work.toString();
                Logger.getInstance().recordOutput("faults" + id, result);

                new PrintCommand("FAULTS: " + result);
            }
            if (errorCode.value != 0)
            {
                Logger.getInstance().recordOutput("faults" + id, "None");
                new PrintCommand("LAST ERROR: " + errorCode.toString() + " (Code " + (errorCode.value) + ")");
            }
        }
    }
}

