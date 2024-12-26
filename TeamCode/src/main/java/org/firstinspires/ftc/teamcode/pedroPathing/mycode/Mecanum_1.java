package org.firstinspires.ftc.teamcode.pedroPathing.mycode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

/**
 * This is an example teleop that showcases robot-centric driving with PIDF control.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@TeleOp(name = "Mecanum_1", group = "Examples")
public class Mecanum_1 extends OpMode {
    private Follower follower;
    private PIDFController pidfController;
    private final Pose startPose = new Pose(0, 0, 0);

    // PIDF constants
    private double kP = 1.0;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kF = 0.0;

    /** This method is called once when init is played, it initializes the follower and PIDF controller. **/
    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize the PIDF controller
        pidfController = new PIDFController(kP, kI, kD, kF);
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        // Get current pose from the follower
        Pose currentPose = follower.getPose();

        // Get the target movement vectors from gamepad input
        double forward = -gamepad1.left_stick_y;   // Forward/backward movement
        double strafe = -gamepad1.left_stick_x;    // Left/right movement
        double rotate = -gamepad1.right_stick_x;   // Rotate left/right

        // Use the PIDF controller to adjust motor speeds based on the error
        // PIDF calculation (this is just an example; you would implement logic to control your motors here)
        double pidfCorrectionX = pidfController.runPIDF(currentPose.getX(), forward);
        double pidfCorrectionY = pidfController.runPIDF(currentPose.getY(), strafe);
        double pidfCorrectionHeading = pidfController.runPIDF(currentPose.getHeading(), rotate);

        // Set the movement vectors with PIDF corrections
        follower.setTeleOpMovementVectors(forward + pidfCorrectionX, strafe + pidfCorrectionY, rotate + pidfCorrectionHeading, true);
        follower.update();

        // Telemetry Outputs of our Follower
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(currentPose.getHeading()));

        // Update Telemetry to the Driver Hub
        telemetry.update();
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}

