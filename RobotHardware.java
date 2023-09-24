package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {

    private LinearOpMode myOpMode = null;


    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorRearLeft = null;
    public DcMotor motorRearRight = null;

    public void init() {
        motorRearLeft = myOpMode.hardwareMap.get(DcMotor.class, "motorRearLeft");
        motorRearRight = myOpMode.hardwareMap.get(DcMotor.class, "motorRearRight");
        motorFrontLeft = myOpMode.hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = myOpMode.hardwareMap.get(DcMotor.class, "motorFrontRight");

        //names

        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorRearRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotor.Direction.REVERSE);
        //Directions

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();

    }

    public void driveRobot(double Drive, double Turn) {


        {
            double left = Drive + Turn;
            double right = Drive - Turn;

            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }


            setDrivePower(left, right);
        }
    }
    public void setDrivePower(double leftWheel, double rightWheel) {
        motorFrontLeft.setPower(leftWheel);
        motorRearLeft.setPower(leftWheel);
        motorFrontRight.setPower(rightWheel);
        motorRearRight.setPower(rightWheel);
    }
}

