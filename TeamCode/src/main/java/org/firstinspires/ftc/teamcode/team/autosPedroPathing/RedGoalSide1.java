package org.firstinspires.ftc.teamcode.team.autosPedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;

/**
 * Pedro Pathing auto using LinearOpMode via DarienOpModeFSM.
 */
@Autonomous(name = "RedGoalSidePedro", group = "Pedro:Reds", preselectTeleOp = "TeleopFSM")
@Configurable
public class RedGoalSide1 extends DarienOpModeFSM {

    private TelemetryManager panelsTelemetry;   // Panels Telemetry instance
    public Follower follower;                   // Pedro Pathing follower instance
    private int pathState;                      // State machine state
    private Paths paths;                        // Paths
    private Timer pathTimer, opmodeTimer;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- ROBOT + HARDWARE INIT (from DarienOpModeFSM) ---
        initControls(); // sets up TrayServo, Elevator, Feeder, motors, AprilTag, etc.

        // --- PEDRO + TIMERS INIT ---
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Starting pose – same as your OpMode version
        follower.setStartingPose(new Pose(121.286, 124.378, Math.toRadians(127)));

        // Build all the paths once
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        telemetry.addLine("RedGoalSidePedro: READY");
        telemetry.update();

        // --- WAIT FOR START ---
        waitForStart();
        if (isStopRequested()) return;

        opmodeTimer.resetTimer();
        setPathState(0);

        // Set the initial tray position immediately.
        setTrayPosition(TRAY_POS_1_SCORE);

        // --- MAIN AUTONOMOUS LOOP ---
        while (opModeIsActive() && !isStopRequested()) {

            // Pedro follower must be updated every loop
            follower.update();

            // Drive the state machine
            pathState = autonomousPathUpdate();

            runIntakeLifterWithColorSensor();

            // Panels/driver telemetry
            panelsTelemetry.addData("Tray Curr", currentTrayPosition);
            panelsTelemetry.addData("Path State", pathState);
            panelsTelemetry.addData("X", follower.getPose().getX());
            panelsTelemetry.addData("Y", follower.getPose().getY());
            panelsTelemetry.addData("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);

            telemetry.update();
        }
    }

    /**
     * Inner class defining all the Pedro paths.
     */
    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;


        //67
        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(121.286, 124.287), new Pose(96.776, 96.443))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(127), Math.toRadians(110))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(96.776, 96.443), new Pose(96.766, 96.443))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(35))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(96.766, 96.443),
                                    new Pose(81.428, 87.768),
                                    new Pose(94.670, 80.567)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(94.670, 80.567), new Pose(104.670, 80.567))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(104.670, 80.567), new Pose(109.500, 80.567))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(109.500, 80.567), new Pose(116.500, 80.567))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(116.500, 80.567),
                                    new Pose(99.436, 103.926),
                                    new Pose(96.443, 119.383)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.443, 96.443),
                                    new Pose(96.423, 122.383)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }

    }

    /**
     * State machine for autonomous sequence.
     * Returns current pathState for logging.
     */
    public int autonomousPathUpdate() {

        // Helpful debug info every loop
        telemetry.addData("PathState", pathState);
        telemetry.addData("FollowerBusy", follower.isBusy());
        telemetry.addData("PathTimer", pathTimer.getElapsedTimeSeconds());

        switch (pathState) {
            case 0:
                telemetry.addLine("Case " + pathState + ": Start Path1");

                // Set the initial tray position
                setTrayPosition(TRAY_POS_1_SCORE);
                follower.setMaxPower(0.8); // move slowly to prevent artifacts from falling out of tray
                follower.followPath(paths.Path1);
                setPathState(pathState + 1);
                break;

            case 1:
                telemetry.addLine("Case " + pathState + ": Wait for Path1 and camera, then start read AprilTag");

                if ((!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.0)) {

                    telemetry.addLine("Case " + pathState + ": exiting");
                    // Start AprilTag reading after path1 is done
                    tagFSM.start(getRuntime());

                    setPathState(pathState + 1);
                }
                break;

            case 2:
                telemetry.addLine("Case " + pathState + ": Read AprilTag then start Path2");

                tagFSM.update(getRuntime(), true, telemetry);

                if ((tagFSM.isDone()) || pathTimer.getElapsedTimeSeconds() > 2.67) {
                    aprilTagDetections = tagFSM.getDetections();

                    telemetry.addLine("Case " + pathState + ": exiting");
                    follower.followPath(paths.Path2);
                    setPathState(pathState + 1);
                }
                break;

            case 3:
                telemetry.addLine("Case " + pathState + ": Wait for Path2, then shoot artifact");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    telemetry.addLine("Case " + pathState + ": exiting");
                    setPathState(pathState + 1);
                }
                break;

            case 4:
                telemetry.addLine("Case " + pathState + ": start shooting");

                shootPatternFSM.startShootPattern(aprilTagDetections, getRuntime(), SHOT_GUN_POWER_UP);

                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    setPathState(pathState + 1);
                }
                break;

            case 5:
                telemetry.addLine("Case " + pathState + ": updateShooting...");
                shootPatternFSM.updateShootPattern(getRuntime());

                if (shootPatternFSM.isShootPatternDone() || pathTimer.getElapsedTimeSeconds() > 10.0) {

                    rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);

                    // now continue with next path
                    follower.followPath(paths.Path3, true);
                    setPathState(pathState + 1);
                }
                break;

            case 6:
                telemetry.addLine("Case " + pathState + ": Wait for Path3, then start Path4");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4.0) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact 1p");

                    follower.setMaxPower(0.175); //slow down for pickup

                    setTrayPosition(TRAY_POS_1_INTAKE);
                    follower.followPath(paths.Path4, true);
                    setPathState(pathState + 1);
                }
                break;

            case 7:
                telemetry.addLine("Case " + pathState + ": Wait for Path4");
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4.2) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact 2p");

                    setTrayPosition(TRAY_POS_3_INTAKE);
                    follower.followPath(paths.Path5, true);
                    setPathState(pathState + 1);
                }
                break;

            case 8:
                telemetry.addLine("Case " + pathState + ": Wait for Path5");
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.3) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact 3g");

                    setTrayPosition(TRAY_POS_2_INTAKE);
                    follower.followPath(paths.Path6, true);
                    setPathState(pathState + 1);
                }
                break;

            case 9:
                telemetry.addLine("Case " + pathState + ": Wait for Path6 to pick up artifact, then start Path7");
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.0) {
                    telemetry.addLine("Case " + pathState + ": Moving to shooting position");

                    follower.setMaxPower(0.5); // move slowly to prevent artifacts from falling out of tray
                    follower.followPath(paths.Path7, true);
                    setTrayPosition(TRAY_POS_1_SCORE);
                    rubberBands.setPower(0);
                    setPathState(pathState + 1);
                }
                break;

            case 10:
                telemetry.addLine("Case " + pathState + ": Wait for Path7 to get into position, then start Path8");
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    telemetry.addLine("Case " + pathState + ": Shoot the pattern");
                    shootPatternFSM.startShootPattern(aprilTagDetections, getRuntime(), SHOT_GUN_POWER_UP * .96875); // 31/32 power
                    setPathState(pathState + 1);
                }
                break;

            case 11:
                telemetry.addLine("Case " + pathState + ": Wait for Path6 to finish, then stop");
                shootPatternFSM.updateShootPattern(getRuntime());
                if (shootPatternFSM.isShootPatternDone()) {
                    telemetry.addLine("Case " + pathState + ": Done, setting state -1");
                    rubberBands.setPower(0);
                    setPathState(-1); // done
                }
                break;

            default:
                // -1 or any undefined state: do nothing, stay idle
                telemetry.addLine("Idle state (pathState = " + pathState + ")");
                break;
        }

        return pathState;
    }

    /**
     * Sets the path state and resets its timer.
     */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}