package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Simple Auto", group = "Examples")
public class SimpleAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Define poses for each position in the path
    // Starting at (72, 72) facing 0 degrees (right)
    private final Pose startPose = new Pose(72, 72, Math.toRadians(0));

    // After moving forward 24 inches
    private final Pose forwardPose = new Pose(96, 72, Math.toRadians(0));

    // After turning left 90 degrees (now facing up) - adding slight movement to help turn
    private final Pose turnPose = new Pose(97, 73, Math.toRadians(90));

    // After moving forward 24 inches (now at higher y position)
    private final Pose forward2Pose = new Pose(97, 97, Math.toRadians(90));

    // After moving backward 24 inches (back to turn position)
    private final Pose backwardPose = new Pose(97, 73, Math.toRadians(90));

    // After strafing left 24 inches (now at lower x position, still facing up)
    private final Pose strafeLeftPose = new Pose(73, 73, Math.toRadians(90));

    // Path chains for each movement
    private PathChain moveForward, turnLeft, moveForward2, moveBackward, strafeLeft;

    public void buildPaths() {
        // Path 1: Move forward 24 inches
        moveForward = follower.pathBuilder()
                .addPath(new BezierLine(startPose, forwardPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        // Path 2: Turn left 90 degrees (in place)
        turnLeft = follower.pathBuilder()
                .addPath(new BezierLine(forwardPose, turnPose))
                .setLinearHeadingInterpolation(forwardPose.getHeading(), turnPose.getHeading())
                .build();

        // Path 3: Move forward 24 inches (now moving in y direction)
        moveForward2 = follower.pathBuilder()
                .addPath(new BezierLine(turnPose, forward2Pose))
                .setConstantHeadingInterpolation(turnPose.getHeading())
                .build();

        // Path 4: Move backward 24 inches
        moveBackward = follower.pathBuilder()
                .addPath(new BezierLine(forward2Pose, backwardPose))
                .setConstantHeadingInterpolation(forward2Pose.getHeading())
                .build();

        // Path 5: Strafe left 24 inches (move in -x direction while facing up)
        strafeLeft = follower.pathBuilder()
                .addPath(new BezierLine(backwardPose, strafeLeftPose))
                .setConstantHeadingInterpolation(backwardPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start moving forward 24 inches
                follower.followPath(moveForward, true);
                setPathState(1);
                break;
            case 1:
                // Wait until forward movement is complete, then turn left
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(turnLeft, true);
                    setPathState(2);
                }
                break;
            case 2:
                // Wait until turn is complete, then move forward again
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(moveForward2, true);
                    setPathState(3);
                }
                break;
            case 3:
                // Wait until forward movement is complete, then move backward
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(moveBackward, true);
                    setPathState(4);
                }
                break;
            case 4:
                // Wait until backward movement is complete, then strafe left
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(strafeLeft, true);
                    setPathState(5);
                }
                break;
            case 5:
                // Wait until strafe is complete, then stop
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(-1); // Stop state
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Telemetry for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("is busy", follower.isBusy());
        telemetry.addData("timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {}
}