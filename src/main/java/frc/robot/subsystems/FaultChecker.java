// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FaultChecker extends SubsystemBase {

    public enum MotorStatus
    {
        OFFLINE(0),
        ONLINE(1),
        WARN(2),
        FATAL(3);

        public final int value;

        MotorStatus(int index) {
            value = index;
        }

        @Override
        public String toString() {
            return this.name();
        }
    }
    
    private class Motor
    {
        private String id;
        private BaseMotorController controller;
        private MotorStatus status;
        private Consumer<MotorStatus> onStatusChanged;

        public Motor(String id, BaseMotorController controller)
        {
            this.id = id;
            this.controller = controller;
        }

        public String getId() { return id; }
        public BaseMotorController getController() { return controller; }
        public MotorStatus getStatus() { return status; }
        public void setStatus(MotorStatus s) { status = s; onStatusChanged.accept(s); }
        public void onStatusChanged(Consumer<MotorStatus> consumer) { onStatusChanged = consumer; }
    }

    private class MotorGroup
    {

        public MotorGroup(String id)
        {
            this.id = id;
            motors = new Motor[4];
        }

        private String id;
        private Motor[] motors;
        private int motorAmount;

        public void addMotor(String id, BaseMotorController controller)
        {
            if (motorAmount >= 4)
                throw new IllegalStateException("You may not add more than 4 motors to a group.");
            Motor m = new Motor(id, controller);
            m.onStatusChanged((newStatus) -> updateSchafferBoard());
            motors[motorAmount] = m;
            motorAmount++;
        }

        public String getId() { return id; }
        public Motor[] getMotors() { return motors; }
        public int getMotorCount() { return motorAmount; }
        private void updateSchafferBoard()
        {
            double[] arr = new double[4];
            for (int i = 0; i < 4; i++)
            {
                if (i < motorAmount)
                    arr[i] = motors[i].getStatus().value;
                else arr[i] = -1;
            }
            SmartDashboard.putNumberArray(id, arr);
        }
    }
    private static FaultChecker instance; 
    // Could potentionally make this
    private int groupIndex = 0;
    private int groupMotorIndex = 0;
    private ArrayList<MotorGroup> motorGroups;
    //private ArrayList<BaseMotorController> motors; 
    /** Creates a new FaultChecker. */
    private FaultChecker() {
        motorGroups = new ArrayList<MotorGroup>();
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
            MotorGroup curGroup = motorGroups.get(groupIndex);
            Motor motor = curGroup.getMotors()[groupMotorIndex];

            checkMotor(motor);
            groupMotorIndex++;
            if (groupMotorIndex >= curGroup.getMotorCount())
            {
                groupMotorIndex = 0;
                groupIndex++;
                if (groupIndex >= motorGroups.size())
                    groupIndex = 0;
            }
        }
    }

    public void addMotor(String id, String group, BaseMotorController motor)
    {
        for (int i = 0; i < motorGroups.size(); i++)
        {
            if (motorGroups.get(i).getId() == group)
            {
                motorGroups.get(i).addMotor(id, motor);
                return;
            }
        }
        MotorGroup newGroup = new MotorGroup(group);
        newGroup.addMotor(id, motor);
        motorGroups.add(newGroup);
    }

    private void checkMotor(Motor motor)
    {
        String id = motor.getId();
        BaseMotorController controller = motor.getController();
        Faults f = new Faults();
        ErrorCode errorCode = controller.getFaults(f);

        Logger.getInstance().recordOutput("lastErrorCode" + id, errorCode.value);
        motor.setStatus(MotorStatus.ONLINE);
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
                motor.setStatus(MotorStatus.FATAL);

                new PrintCommand("FAULTS: " + result);
            }
            if (errorCode.value != 0)
            {
                Logger.getInstance().recordOutput("faults" + id, "None");
                new PrintCommand("LAST ERROR: " + errorCode.toString() + " (Code " + (errorCode.value) + ")");
                motor.setStatus(MotorStatus.WARN);
            }
        }
    }
}

