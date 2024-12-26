package org.firstinspires.ftc.teamcode.pedroPathing.mycode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@TeleOp(name = "Example_Mecanum", group = "Examples")
public class Example_Mecanum extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */


        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        /* This could be paired with a PIDF to set the target position of the lift in teleop.
         * For this, you would have to update the lift pid and make sure to initializes the lift subsystem.
         */

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));


        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    @Override
    public void stop() {
    }
}
