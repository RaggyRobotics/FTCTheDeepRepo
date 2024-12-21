package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.FtcDashboard;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MecanumDrive", group = "TeleOp")
public class MecanumDrive extends OpMode{

    private DcMotor leftFront,rightFront, leftRear, rightRear;

    private final double Kp = 0;
    private final double Ki = 0;
    private final double Kd = 0;

    private double[] lastError = {0,0,0,0};
    private double[] integral = {0,0,0,0};

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double leftFrontPower = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftRearPower = drive - strafe + turn;
        double rightRearPower = drive + strafe - turn;

        leftFrontPower = pidControl(leftFrontPower,leftFront.getCurrentPosition(), (0));
        rightFrontPower = pidControl(rightFrontPower,rightFront.getCurrentPosition(), (0));
        leftRearPower = pidControl(leftRearPower,leftRear.getCurrentPosition(), (0));
        rightRearPower = pidControl(rightRearPower,rightRear.getCurrentPosition(), (0));

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);

        telemetry.addData("Front Left Power",leftFrontPower);
        telemetry.addData("Front Right Power",rightFrontPower);
        telemetry.addData("Back Left Power",leftRearPower);
        telemetry.addData("Back Right Power",rightRearPower);
    }

    private double pidControl(double targetPower, int currentPosition, int motorIndex) {
        double targetPosition = targetPower * 1000;
        double error = targetPosition - currentPosition;

        integral[motorIndex] += error;
        double derivative = error - lastError[motorIndex];

        double output = (Kp * error) + (Ki * integral[motorIndex]) + (Kd * derivative);

        lastError[motorIndex] = error;

        return Range.clip(output, -1, 1);
    }
}