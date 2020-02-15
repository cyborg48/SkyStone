/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
// Based on HardwarePushbot
public class Hardware6417
{
    /* Public OpMode members. */
    public DcMotorEx leftFront = null, rightFront = null, leftBack = null, rightBack = null, leftSpin = null, rightSpin = null;

    // armMotor = null,

    public Servo liftServo = null, latchServo = null, dragServo = null;

    ColorSensor colorSensor;
    ModernRoboticsI2cGyro alignGyro;
    IntegratingGyroscope gyro;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    public static final double TURN_POWER_LIFT =  0.5 ;
    public static final int CPR = 1440;
    public static final double DIAMETER = 3.93701; //inches
    //public static final int BLOCK_HEIGHT = 127; //millimeters

    BNO055IMU imu;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware6417(){
    }

    private void checkOrientation() {
        // read the orientation of the robot
        Orientation angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        // and save the heading
        double curHeading = angles.firstAngle;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and initialize motors
        leftFront = hwMap.get(DcMotorEx.class, "FrontLeft");
        leftBack = hwMap.get(DcMotorEx.class, "BackLeft");
        rightFront = hwMap.get(DcMotorEx.class, "FrontRight");
        rightBack = hwMap.get(DcMotorEx.class, "BackRight");
        leftSpin = hwMap.get(DcMotorEx.class, "LeftSpin");
        rightSpin = hwMap.get(DcMotorEx.class, "RightSpin");


        liftServo = hwMap.get(Servo.class, "liftServo");
        latchServo = hwMap.get(Servo.class, "latchServo");

        // Set motor and servo directions based on orientation of motors on robot
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftSpin.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightSpin.setDirection(DcMotor.Direction.REVERSE);
        //armMotor.setDirection(DcMotor.Direction.FORWARD);
        //extendMotor.setDirection(DcMotor.Direction.FORWARD);

        liftServo.setDirection(Servo.Direction.FORWARD);
        latchServo.setDirection(Servo.Direction.FORWARD);

        // Set initial servo positions
        //alignServo.setPosition(1.0);
        //dragServo.setPosition(0);

        liftServo.setPosition(0);
        latchServo.setPosition(0);

        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // Set all motors to run with encoders
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

    }

    public void drivetoPosition(int d, double power){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int distance = d;
        //int distance = (int)(CPR / (DIAMETER * Math.PI) * d);

        leftFront.setTargetPosition(distance);
        rightFront.setTargetPosition(distance);
        leftBack.setTargetPosition(distance);
        rightBack.setTargetPosition(distance);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);

        while(leftBack.isBusy()){

        } // || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy()

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    public void strafeToPosition(int d, double power){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int distance = d;
        //int distance = (int)(CPR / (DIAMETER * Math.PI) * d);

        leftFront.setTargetPosition(distance);
        rightFront.setTargetPosition(-distance);
        leftBack.setTargetPosition(-distance);
        rightBack.setTargetPosition(distance);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(power);

        while(leftBack.isBusy()){ }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setDriveSpeeds(double forward, double strafe, double rotate, double correction) {

        double frontLeftSpeed = forward + strafe + rotate - correction; //-correction
        double frontRightSpeed = forward - strafe - rotate + correction; //+correction
        double backLeftSpeed = forward - strafe + rotate - correction; //-correction
        double backRightSpeed = forward + strafe - rotate + correction; //+correction

        double largest = 1.0;
        largest = Math.max(largest, Math.abs(frontLeftSpeed));
        largest = Math.max(largest, Math.abs(frontRightSpeed));
        largest = Math.max(largest, Math.abs(backLeftSpeed));
        largest = Math.max(largest, Math.abs(backRightSpeed));

        leftFront.setPower(frontLeftSpeed / largest);
        rightFront.setPower(frontRightSpeed / largest);
        leftBack.setPower(backLeftSpeed / largest);
        rightBack.setPower(backRightSpeed / largest);

        if(rotate!=0){
            resetAngle();
        }

    }

    public void stop(){

        // Set all motors to 0 power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

    }


    public void lift(double lift){
        //armMotor.setPower(lift);
        double armangle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        /***
        if(lift < 0){ // going up
            alignServo.setDirection(Servo.Direction.REVERSE);
            //robot.alignServo.setPosition(robot.alignServo.getPosition() - 0.001);
            if(alignServo.getPosition() <= 1) {
                alignServo.setPosition(0 + ((0.3/60) * armangle));
            }
        }
        else if (lift > 0){ // going down
            alignServo.setDirection(Servo.Direction.FORWARD);
            //robot.alignServo.setPosition(robot.alignServo.getPosition() + 0.001);
            if(alignServo.getPosition() >= 0){
                alignServo.setPosition(0 - ((0.3/60) * armangle));
            }
         ***/
        }



    public void liftWithEncoder(double d, double power){

        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armMotor.setPower(power);

            /***

            double armangle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            if(d < 0){ // going up
                alignServo.setDirection(Servo.Direction.FORWARD);
                //robot.alignServo.setPosition(robot.alignServo.getPosition() - 0.001);
                if(alignServo.getPosition() <= 1) {
                    alignServo.setPosition(0.7 + ((0.3/60) * armangle));
                }
            }
            else if (d > 0){ // going down
                alignServo.setDirection(Servo.Direction.REVERSE);
                //robot.alignServo.setPosition(robot.alignServo.getPosition() + 0.001);
                if(alignServo.getPosition() >= 0){
                    alignServo.setPosition(0.3 - ((0.3/60) * armangle));
                }
            }
             ***/


        //armMotor.setPower(0);
        //armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .01;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void spin(){

        rightSpin.setPower(0.7);
        leftSpin.setPower(0.7);

    }


}