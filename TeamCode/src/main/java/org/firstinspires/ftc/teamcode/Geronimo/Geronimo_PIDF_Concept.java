package org.firstinspires.ftc.teamcode.Geronimo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
 * This file contains example PIDF control code inspired by KookyBotz PIDF & Arm Control Video.
 * The Dashboard is used to set rotator arm target position and p, i, d, and f values to calibrate
 * the PIDF controller to smoothly set and hold the two rotator motors on Geromino.
 */


/* Changes to gradle files:
 *
 * build.common.gradle:
 *   repositories {
 *   mavenCentral()      //added for FTC Lib library access (1/1/2025)
 * }
 *
 * build.dependencies.gradle:
 *   repositories {
 *   maven { url ='https://maven.brott.dev'}  //added for FTC Dashboard access (1/1/2025)
 * }
 *
 * build.gradle:
 * dependencies {
 * implementation "org.ftclib.ftclib:core:2.1.1"  //added for FTC Lib library access (1/1/2025)
 * }
 */

@Config
@TeleOp(name="Geronimo: Rotator Arm PIDF Control Concept")
//@Disabled
public class Geronimo_PIDF_Concept extends LinearOpMode {

    public static double p = 0, i = 0, d = 0, f = 0;

    public static int rotator_arm_angle = 0; // target arm angle

    DcMotor leftSlideArmRotatorMotor;
    DcMotor rightSlideArmRotatorMotor;

    @Override
    public void runOpMode() {
        final double arm_gear_ratio = 90.0/20.0;
        final double yellow_jacket_27_ticks = 751.8;    //9.4 ticks for each degree of arm rotation
        final double yellow_jacket_51_ticks = 1425.1;   //17.81 ticks for each degree of arm rotation
        final double ticks_in_degrees = (arm_gear_ratio/360.0) * yellow_jacket_27_ticks;

        double rotator_arm_target = rotator_arm_angle * ticks_in_degrees;

        //Initialize PID Controllers for arm rotator motors.
        PIDController controller = new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app).

        leftSlideArmRotatorMotor = hardwareMap.get(DcMotorEx.class, "leftRotater");
        rightSlideArmRotatorMotor = hardwareMap.get(DcMotorEx.class, "rightRotater");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.

        leftSlideArmRotatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlideArmRotatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            controller.setPID(p,i,d);
            rotator_arm_target = rotator_arm_angle * ticks_in_degrees;
/*
            //change target value...
             if(gamepad2.dpad_left){
                rotator_arm_angle++;
            }
            else if(gamepad2.dpad_right){
                rotator_arm_angle--;
            }

            //change p
            else if (gamepad2.dpad_up){
                p = p + 0.0001;
             }
             else if (gamepad2.dpad_down){
                 p = p - 0.0001;
             }

             //change f
             else if (gamepad2.circle){
                 f = f + 0.0005;
             }
             else if (gamepad2.square){
                 f = f - 0.0005;
             }

 */


            int left_armPos = leftSlideArmRotatorMotor.getCurrentPosition();
            double left_pid = controller.calculate(left_armPos,rotator_arm_target);
            double left_ff = Math.cos(Math.toRadians(rotator_arm_target/ticks_in_degrees)) * f;

            int right_armPos = leftSlideArmRotatorMotor.getCurrentPosition();
            double right_pid = controller.calculate(right_armPos,rotator_arm_target);
            double right_ff = Math.cos(Math.toRadians(rotator_arm_target/ticks_in_degrees)) * f;


            // Setup a variable for each arm rotator motor to save power level for telemetry
            double leftPower = left_pid + left_ff;
            double rightPower = right_pid + right_ff;

            // TODO: Driver input - use joystick controls to set rotator_arm_target value
            //  that is limited between Min and Max values

            // Send calculated power to motors
            leftSlideArmRotatorMotor.setPower(leftPower);
            leftSlideArmRotatorMotor.setPower(rightPower);

            // Show arm target, arm positions, and arm power.
            telemetry.addData("Rotator arm target", rotator_arm_target);
            //telemetry for rotator_arm_angle
            telemetry.addData("Rotator arm angle", rotator_arm_angle);
            telemetry.addData("Rotator positions", "left (%d), right (%d)", left_armPos, right_armPos);
            telemetry.addData("Rotator motor power", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}