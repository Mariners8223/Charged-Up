package frc.util.subsystemTemplates;

import org.littletonrobotics.junction.AutoLog;

public interface TemplateIO {
    @AutoLog
    public static class templateIOInputs {}

    public default void updateInputs(templateIOInputs inputs) {}

    public default void setVoltage(double volts) {}
}
