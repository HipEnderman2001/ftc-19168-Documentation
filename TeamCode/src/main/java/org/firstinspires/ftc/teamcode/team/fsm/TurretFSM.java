package org.firstinspires.ftc.teamcode.team.fsm;

import static com.pedropathing.math.MathFunctions.clamp;

import com.qualcomm.robotcore.hardware.Servo;

public class TurretFSM {
    private final DarienOpModeFSM opMode;
    private final Servo turretServo;

    private double currentTurretPosition;

    /**
     * Constructor for TurretFSM
     * @param opMode The main op mode instance
     */
    public TurretFSM(DarienOpModeFSM opMode, Servo turretServo) {
        this.turretServo = turretServo;
        this.opMode = opMode;
    }

    /**
     * Get the current turret heading in degrees based on the servo position.
     * Zero heading is straight ahead (forward motion of the robot)
     * Positive degrees indicate left, negative degrees indicate right.
     * @return The turret heading in degrees.
     */
    public double getTurretHeading(){
        return (double) -DarienOpModeFSM.FIVE_ROTATION_SERVO_SPAN_DEG /2 + DarienOpModeFSM.FIVE_ROTATION_SERVO_SPAN_DEG * currentTurretPosition;
    }

    /**
     * Set the turret servo position and save it to memory.
     * @param position The desired servo position (0.0 to 1.0).
     */
    public void setTurretServoPosition(double position) {
        currentTurretPosition = clamp(position, 0, 1);
        turretServo.setPosition(currentTurretPosition);
    }

    /**
     * Get the current turret servo position.
     * @return The current turret servo position (0.0 to 1.0).
     */
    public double getCurrentTurretServoPosition() {
        return currentTurretPosition;
    }

}


