/* ============================================================= *
 *        Pedro Pathing Plus Visualizer — Auto-Generated         *
 *                                                               *
 *  Version: 1.7.2.                                              *
 *  Copyright (c) 2026 Matthew Allen                             *
 *                                                               *
 *  THIS FILE IS AUTO-GENERATED — DO NOT EDIT MANUALLY.          *
 *  Changes will be overwritten when regenerated.                *
 * ============================================================= */

package org.firstinspires.ftc.teamcode.Commands.AutoCommands;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private ElapsedTime pathTimer; // Timer for path state machine
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        // ...
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 9, Math.toRadians(180)));

        pathTimer = new ElapsedTime();
        paths = new Paths(follower); // Build paths
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain line1;
        public PathChain line2;
        public PathChain line3;
        public PathChain line4;
        public PathChain line5;
        public PathChain line6;
        public PathChain line7;
        public PathChain line8;
        public PathChain line9;
        public PathChain line10;

        public Paths(Follower follower) {
            line1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 9.000), new Pose(56.000, 60.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .setReversed()
                    .build();

            line2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 60.000), new Pose(23.889, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            line3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(23.889, 60.000), new Pose(21.000, 66.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.000, 66.000), new Pose(56.000, 17.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 17.000), new Pose(56.000, 36.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 36.000), new Pose(21.000, 36.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            line7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.000, 36.000), new Pose(56.000, 9.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 9.000), new Pose(13.000, 9.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            line9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(13.000, 9.000), new Pose(56.000, 9.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 9.000), new Pose(27.000, 9.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setPathState(1);
                break;
            case 1:
                if (pathTimer.milliseconds() > 1500) {
                    setPathState(2);
                }
                break;
            case 2:
                follower.followPath(paths.line1, true);
                setPathState(3);
                break;
            case 3:
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;
            case 4:
                follower.followPath(paths.line2, true);
                setPathState(5);
                break;
            case 5:
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
                follower.followPath(paths.line3, true);
                setPathState(7);
                break;
            case 7:
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case 8:
                setPathState(9);
                break;
            case 9:
                if (pathTimer.milliseconds() > 500) {
                    setPathState(10);
                }
                break;
            case 10:
                follower.followPath(paths.line4, true);
                setPathState(11);
                break;
            case 11:
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;
            case 12:
                setPathState(13);
                break;
            case 13:
                if (pathTimer.milliseconds() > 1500) {
                    setPathState(14);
                }
                break;
            case 14:
                follower.followPath(paths.line5, true);
                setPathState(15);
                break;
            case 15:
                if (!follower.isBusy()) {
                    setPathState(16);
                }
                break;
            case 16:
                follower.followPath(paths.line6, true);
                setPathState(17);
                break;
            case 17:
                if (!follower.isBusy()) {
                    setPathState(18);
                }
                break;
            case 18:
                follower.followPath(paths.line7, true);
                setPathState(19);
                break;
            case 19:
                if (!follower.isBusy()) {
                    setPathState(20);
                }
                break;
            case 20:
                setPathState(21);
                break;
            case 21:
                if (pathTimer.milliseconds() > 1500) {
                    setPathState(22);
                }
                break;
            case 22:
                follower.followPath(paths.line8, true);
                setPathState(23);
                break;
            case 23:
                if (!follower.isBusy()) {
                    setPathState(24);
                }
                break;
            case 24:
                follower.followPath(paths.line9, true);
                setPathState(25);
                break;
            case 25:
                if (!follower.isBusy()) {
                    setPathState(26);
                }
                break;
            case 26:
                setPathState(27);
                break;
            case 27:
                if (pathTimer.milliseconds() > 1500) {
                    setPathState(28);
                }
                break;
            case 28:
                follower.followPath(paths.line10, true);
                setPathState(29);
                break;
            case 29:
                if (!follower.isBusy()) {
                    setPathState(30);
                }
                break;
            case 30:
                requestOpModeStop();
                pathState = -1;
                break;
        }
        return pathState;
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }
}
