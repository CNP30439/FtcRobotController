package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

@Autonomous(name = "Forward 24 Then Turn 180", group = "Test")
@Configurable
public class ForwardThenTurnTest extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private PathChain driveForward;
    private PathChain turnRight;
    private Timer pathTimer;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Path 1: Drive forward 24 inches, keep heading at 0 the whole time
        driveForward = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(0, 0),
                                new Pose(24, 0)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 2: Move a tiny bit forward (0.1 inch) so Pedro has path length,
        // while sweeping heading from 0 to -180 (right turn)
        turnRight = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(24, 0),
                                new Pose(24.1, 0)  // tiny movement = gives path distance
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-180))
                .build();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {
            case 0:
                follower.followPath(driveForward, true);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    pathState = 2;
                }
                break;

            case 2:
                follower.followPath(turnRight, true);
                pathState = 3;
                break;

            case 3:
                if (!follower.isBusy()) {
                    pathState = -1;
                }
                break;
        }

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {}
}