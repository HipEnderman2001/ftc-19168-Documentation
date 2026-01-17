package org.firstinspires.ftc.teamcode.team.fsm;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ShotgunFSM {

    private final DarienOpModeFSM opmode;
    private final double SHOT_GUN_POWER_UP;
    private final double SHOT_GUN_POWER_UP_FAR;
    private final DcMotorEx shotgunMotor;

    public enum State {
        OFF,
        POWER_UP,
        POWER_UP_FAR;
    }

    private State current = State.OFF;

    public ShotgunFSM(double powerlow, double powerhigh, DcMotorEx motor, DarienOpModeFSM opmode) {
        this.SHOT_GUN_POWER_UP = powerlow;
        this.SHOT_GUN_POWER_UP_FAR = powerhigh;
        this.shotgunMotor = motor;
        this.opmode = opmode;
    }

    /** Set the desired state explicitly. */
    public void setState(State state) {
        this.current = state;
    }

    /** Get the current state. */
    public State getState() {
        return current;
    }

    /** Convenience methods for transitions. */
    public void toOff() {
        current = State.OFF;
        shotgunMotor.setPower(0);
    }

    public void toPowerUp() {
        current = State.POWER_UP;
       // shotgunMotor.setPower(opmode.getVoltageAdjustedMotorPower(SHOT_GUN_POWER_UP));
        shotgunMotor.setVelocity(opmode.getTicksPerSecond(opmode.SHOT_GUN_POWER_UP_RPM));
    }

    public void toPowerUpFar(double power) {
        current = State.POWER_UP_FAR;
       // shotgunMotor.setPower(opmode.getVoltageAdjustedMotorPower(SHOT_GUN_POWER_UP_FAR));
        shotgunMotor.setVelocity(opmode.getTicksPerSecond(power));
    }

}
