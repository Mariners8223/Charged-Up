// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.humanIO;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/**  
 * An Axis that gets its state from {@link State}
 * 
 * <p>This class exists in the util>humanIO directory
*/
public class JoystickAxis extends Trigger {
    CommandPS5Controller c;
    int axis;
    EventLoop loop;
    /**
     * Creates a JoystickAxis instance.
     * @param axis The axis (see {@link PS5Controller.Axis}) to bind this JoystickAxis to.
     */
    public JoystickAxis(CommandPS5Controller c, int axis)
    {
        super(() -> c.getRawAxis(axis) > 0.1); // onTrue and onFalse return 
        this.c = c;
        this.axis = axis;
        loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    }

    public JoystickAxis(EventLoop loop, CommandPS5Controller c, int axis)
    {
        super(loop, () -> c.getRawAxis(axis) > 0.1); // onTrue and onFalse 
        this.c = c;
        this.axis = axis;
        this.loop = loop;
    }

    /**
     * @return the current state of the trigger.
     */
    public State getCurrentState()
    {
        if (c.getRawAxis(axis) > Constants.fullThres)
            return State.fullyHeld;
        else if (c.getRawAxis(axis) > Constants.heldThres)
            return State.held;
        return State.letGo;
    }

    /**
     * When the trigger is held fully (goes above <code>Constants.fullThres</code>), calls a command.
     * @param c The controller of which to bind the command to.
     * @param axis The axis of the trigger to bind the command to.
     * @param command The command to call when the trigger is fully held.
     */
    public void onFullyHeld(Command command)
    {
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(
            new Runnable() {
                private boolean m_pressedLast = false;

                @Override
                public void run() {
                boolean pressed = c.getRawAxis(axis) > Constants.fullThres;

                if (!m_pressedLast && pressed) {
                    command.schedule();
                }

                m_pressedLast = pressed;
                }
            });
    }

    /**
     * When the trigger is held fully (goes above <code>Constants.fullThres</code>), calls a command.
     * When the trigger is released, cancels the command.
     * @param command The command to call when the trigger is fully held.
     */
    public void whileFullyHeld(Command command)
    {
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(
            new Runnable() {
                private boolean m_pressedLast = false;

                @Override
                public void run() {
                boolean pressed = c.getRawAxis(axis) > Constants.fullThres;

                if (!m_pressedLast && pressed) {
                    command.schedule();
                } else if (m_pressedLast && !pressed) {
                    command.cancel();
                }

                m_pressedLast = pressed;
                }
            });
    }

    /**
     * When the trigger is held (goes above <code>Constants.heldThres</code>, calls a command.
     * @param command The command to call when the trigger is held.
     */
    @Override
    public Trigger onTrue(Command command) {
        return super.onTrue(command);
    }

    /**
     * When the trigger is held (goes above <code>Constants.heldThres</code>, calls a command.
     * When the trigger is let go, cancels the command.
     * @param command The command to call when the trigger is held.
     */
    @Override
    public Trigger whileTrue(Command command) {
        return super.whileTrue(command);
    }

    /**
     * When the trigger is completely let go (drops below <code>Constants.heldThres</code>), calls a command.<br>
     * @param command The command to call when the trigger is let go.
     */
    @Override
    public Trigger onFalse(Command command) {
        return super.onFalse(command);
    }

    /**
     * When the trigger is completely let go (drops below <code>Constants.heldThres</code>), calls a command.<br>
     * When the trigger is held again, cancels the command.
     * @param command The command to call when the trigger is let go.
     */
    @Override
    public Trigger whileFalse(Command command) {
        return super.whileFalse(command);
    }
    /**
     * When the trigger is loose (drops below <code>Constants.fullThres</code>), calls a command.
     * @param command The command to call when the trigger is released.
     */
    public void onLoosened(Command command)
    {
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(
            new Runnable() {
                private boolean m_releasedLast = true; // defaults to "true" so we don't call the command on robot init.

                @Override
                public void run() {
                boolean pressed = c.getRawAxis(axis) < Constants.fullThres;

                if (!m_releasedLast && pressed) {
                    command.schedule();
                }

                m_releasedLast = pressed;
                }
            });
    }

    /**
     * When the trigger is let loose (drops below <code>Constants.fullThres</code>), calls a command.
     * When the trigger is fully held again, cancels the same command.
     * @param command The command to call when the trigger is released.
     */
    public void whileLoosened(Command command)
    {
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(
            new Runnable() {
                private boolean m_releasedLast = true; // defaults to "true" so we don't call the command on robot init.

                @Override
                public void run() {
                boolean pressed = c.getRawAxis(axis) < Constants.fullThres;

                if (!m_releasedLast && pressed) {
                    command.schedule();
                }
                else if (m_releasedLast && !pressed) {
                    command.cancel();
                }

                m_releasedLast = pressed;
                }
            });
    }
}