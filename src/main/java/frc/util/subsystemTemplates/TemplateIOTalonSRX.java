package frc.util.subsystemTemplates;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.FaultChecker;

public class TemplateIOTalonSRX implements TemplateIO {
    // private static final double GEAR_RATIO = 1.0;
    // private static final double TICKS_PER_REV = 1.0;

    private final TalonSRX leader;
    private final TalonSRX follower;

    private TemplateIOTalonSRX() {
        leader = new TalonSRX(0);
        follower = new TalonSRX(1);

        FaultChecker.getInstance().addMotor("Template Leader", "Template Group", leader);
        FaultChecker.getInstance().addMotor("Template Follower", "Template Group", follower);
        
        leader.configVoltageCompSaturation(12);
        follower.configVoltageCompSaturation(12);
        follower.follow(leader);
        leader.setInverted(false);
        follower.setInverted(InvertType.FollowMaster);
    }


    @Override
    public void updateInputs(templateIOInputs inputs) {
        DriverStation.reportError("No inputs", false);
        throw new Error("No Inputs");
    }
}
