//Single driver field centric for the red alliance

package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Single Driver red", group = "Taus")
//@Config

public class TeleopSRed extends LinearOpMode {

    public AutonomousMethods method = new AutonomousMethods();
    boolean isAPressed = false;
    boolean shooterOn = true;
    boolean isBPressed = false;
    boolean clawClosed = true;
    boolean isRunning = false;
    boolean isXPressed = false;
    boolean dpadPressed = false;
    boolean leftStick = false;
    boolean accelerating = true;
    boolean isYPressed = false;
    boolean isBlockerDown = true;
    boolean RingIn = false;
    boolean intakingRing = true;
    double rings = 0;
    boolean shooting = false;

    double multiplier = 1;
    double speedFactor = 1;
    double previousY = 0;
    double previousX = 0;
    double prevMagnitude = 0;
    boolean firstShot = true;
    boolean intakingInitially;
    double startingRpm = method.staticShooterRpm;
    double startingAngle = -10;
    double incrementRpm = 6;
    double incrementAngle = -.3;
    double scale = 1;

    @Override
    //run file
    public void runOpMode() {
        method.robot.initializeHardware(hardwareMap);
        telemetry.addLine(method.magic8());
        telemetry.update();

        method.robot.shooter.setVelocityPIDFCoefficients(method.p,method.i,method.d,method.f);
        method.resetAngle = method.getHeading();
        while(!isStarted()) {
            telemetry.addData("system", method.robot.imu.getSystemStatus());
            telemetry.addData("status", method.robot.imu.getCalibrationStatus());
            telemetry.addLine(method.magic8());
            telemetry.update();
        }

        waitForStart();
        method.setShooterPower(method.shooterPower);
        method.controlIndexServo(1);
        method.controlBlocker(0);
        method.controlArmServo(0);//move arm up
        method.controlClawServo(.25);//open

        method.runtime2.reset();
        method.runtime3.reset();
        while (opModeIsActive()) {
            drive();

            shoot();
            intake();
            claw();
            indexer();
            blocker();

            //shoot();
            //updateShootingParameters();
            toAngle();
            powerShot();

            //goToPosition();
            //updatePosition();
            resetAngle();
            ringIn();

            telemetry.addData("angle", (int)method.getHeading());
            telemetry.addData("target", (int)method.shooterRpm);
            telemetry.addData("rpm", (int)(method.robot.shooter.getVelocity()/28.0)*60);
            telemetry.addData("numRings", rings);
            telemetry.addData("time", method.runtime3.seconds());
            telemetry.addData("position", "[" +(int)method.currentXPosition + ", " + (int)method.currentYPosition + "]");
            telemetry.update();
            telemetry.clear();

        }

        method.setAllMotorsTo(0);
    }

    //drive base movement
    public void drive(){
        method.runWithEncoders();
        if (gamepad1.right_stick_button&&!leftStick){
            if (speedFactor==1){
                speedFactor = .25;
            }
            else {
                speedFactor = 1;
            }
            leftStick = true;
        }
        if(!gamepad1.right_stick_button){
            leftStick = false;
        }
        double scaleFactor = 1;
        double rotationValue = 0;
        double stickX = 0;
        double stickY = 0;

        if(Math.abs(gamepad1.right_stick_x)>.05) {
            rotationValue = gamepad1.right_stick_x;
        }
        else{
            rotationValue=0;
        }
        if(Math.abs(gamepad1.left_stick_x)>.05) {
            stickX = -gamepad1.left_stick_x;
        }
        else {
            stickX=0;
        }
        if(Math.abs(gamepad1.left_stick_y)>.05) {
            stickY = gamepad1.left_stick_y;
        }
        else {
            stickY=0;
        }
        double gyroAngle = method.getHeading() * Math.PI / 180; //Converts gyroAngle into radians

        //Robot Centric
        //gyroAngle = Math.PI / 2;

        //inverse tangent of game-pad stick y/ game-pad stick x = angle of joystick
        double joystickAngle = Math.atan2(stickY, stickX);
        double theta =  joystickAngle+gyroAngle;

        //changing from a [+] with -- being y and | being x to an [X] with \ being y and / being x (left is forward)
        double calculationAngle = theta - ((3*Math.PI) / 4);

        //magnitude of movement using pythagorean theorem
        double magnitude = Math.sqrt(Math.pow(stickX, 2) + Math.pow(stickY, 2));
        double xComponent = magnitude * (Math.cos(calculationAngle));
        double yComponent = magnitude * (Math.sin(calculationAngle));

        //creates scaleFactor to make sure movement+turning doesn't exceed power 1
        if (yComponent - rotationValue > 1) {
            scaleFactor = Math.abs(yComponent - rotationValue);
        }
        if (yComponent + rotationValue > 1 && yComponent + rotationValue > scaleFactor) {
            scaleFactor = Math.abs(yComponent + rotationValue);
        }

//        if(method.runtime4.seconds()>.25) {
//            if (Math.abs(magnitude - prevMagnitude) / method.runtime4.seconds() > 1) {
//                scale = magnitude - prevMagnitude;
//                method.runtime2.reset();
//            }
//            method.runtime4.reset();
//            prevMagnitude = magnitude*multiplier;
//        }

//        multiplier = method.errorToPower(method.runtime2.seconds(), scale, 0, 1, 0);

        method.robot.frontLeftMotor.setPower((((xComponent + rotationValue) / scaleFactor)*multiplier)*speedFactor);
        method.robot.backRightMotor.setPower((((xComponent - rotationValue) / scaleFactor)*multiplier)*speedFactor);//x
        method.robot.backLeftMotor.setPower((((yComponent + rotationValue) / scaleFactor)*multiplier)*speedFactor);//y
        method.robot.frontRightMotor.setPower((((yComponent - rotationValue) / scaleFactor)*multiplier)*speedFactor);

    }

    //shooting
    public void shooter(){
        if(gamepad1.x && !isXPressed){
            isXPressed = true;
            if (!shooterOn) {
                method.setShooterPower(method.shooterPower);
                shooterOn = true;
            }
            else{
                method.setShooterPower(0);
                shooterOn = false;
            }
        }
        /*if(gamepad1.dpad_up && !dpadPressed){
            method.shooterRpm +=50;
            method.shooterPower = (method.shooterRpm*28)/60.0;
            method.setShooterPower(method.shooterPower);
            dpadPressed=true;
        }
        else if(gamepad1.dpad_down && !dpadPressed){
            method.shooterRpm-=50;
            method.shooterPower = (method.shooterRpm*28)/60.0;
            method.setShooterPower(method.shooterPower);
            dpadPressed=true;
        }
        else if(gamepad1.dpad_right && !dpadPressed){
            method.shooterRpm=method.staticShooterRpm;
            method.shooterPower = (method.shooterRpm*28)/60.0;
            method.setShooterPower(method.shooterPower);
            dpadPressed=true;
        }*/

        if(!gamepad2.x){
            isXPressed = false;
        }
        //if(!gamepad1.dpad_up && !gamepad2.dpad_down && !gamepad2.dpad_right){
        //    dpadPressed = false;
        //}
    }
    public void toAngle(){
        if(gamepad1.left_bumper){
            method.currentXPosition = 12;
            method.currentYPosition = 72;
//            method.shootingAngle = 0;
//            method.shooterRpm = 2135;
//            method.shooterPower = (method.shooterRpm*28)/60.0;
//            method.setShooterPower(method.shooterPower);
            updateShootingParameters();
            method.toAngle(method.shootingAngle, 1);
            shooting = true;
        }
        if(gamepad1.dpad_down){
            method.currentXPosition = 36;
            method.currentYPosition = 72;
//            method.shootingAngle = -12;
//            method.shooterRpm = 2135;
//            method.shooterPower = (method.shooterRpm*28)/60.0;
//            method.setShooterPower(method.shooterPower);
            updateShootingParameters();
            method.toAngle(method.shootingAngle, 1);
            shooting = true;
        }
        if(gamepad1.right_bumper){
            method.currentXPosition = 60;
            method.currentYPosition = 72;
//            method.shootingAngle = -25;
//            method.shooterRpm = 2135;
//            method.shooterPower = (method.shooterRpm*28)/60.0;
//            method.setShooterPower(method.shooterPower);
            updateShootingParameters();
            method.toAngle(method.shootingAngle, 1);
            shooting = true;
        }
        if (shooting&&!(Math.abs(gamepad1.right_stick_x)>.1)){
            if(Math.abs(method.getHeading()-method.shootingAngle)>1) {
                method.toAngle(method.shootingAngle, 1);
            }
        }
        else{
            shooting=false;
        }
    }
    public void powerShot(){
//        if(gamepad1.left_trigger>.1) {
//            telemetry.addLine(method.magic8());
//            telemetry.update();
//            method.powerShot(-15, -11, -6, method.powerShotPower, method.shooterPower);
//            rings=0;
//        }
        if(gamepad1.dpad_left){
            method.setShooterPower(method.powerShotPower);
            method.toAngle(-15, .5);
            method.shootRings(1);
            method.setShooterPower(method.shooterPower);
            rings--;
        }
        if(gamepad1.dpad_up){
            method.setShooterPower(method.powerShotPower);
            method.toAngle(-11, .5);
            method.shootRings(1);
            method.setShooterPower(method.shooterPower);
            rings--;
        }
        if(gamepad1.dpad_right){
            method.setShooterPower(method.powerShotPower);
            method.toAngle(-6, .5);
            method.shootRings(1);
            method.setShooterPower(method.shooterPower);
            rings--;
        }
    }

    //subsystems and utilities
    public void indexer(){
        if(gamepad1.a && !isAPressed){
            isAPressed = true;
            method.shootRings(1);
            if (rings>0) {
                rings -= 1;
            }
        }
        if(!gamepad1.a){
            isAPressed = false;
        }
    }
    public void blocker(){
        if(gamepad2.y && !isYPressed){
            isYPressed = true;
            if (isBlockerDown) {
                method.controlBlocker(0);
                isBlockerDown = false;
            }
            else{
                method.controlBlocker(1);
                isBlockerDown = true;
            }
        }
        if(!gamepad2.y){
            isYPressed = false;
        }
        if(method.runtime3.seconds()>90&&method.runtime3.seconds()<93){
            method.controlBlocker(1);
            isBlockerDown = true;
        }
    }
    public void intake(){
        method.robot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(gamepad1.left_trigger>.05&&rings<3) {
            //method.setIntakePower(-gamepad2.left_stick_y);
            method.setIntakePower(-1);
            intakingRing=false;
        }
        else if (gamepad1.right_trigger>.05&&rings<3) {
            //method.setIntakePower(-gamepad2.left_stick_y);
            method.setIntakePower(1);
            intakingRing=true;
        }
        else if(gamepad1.right_trigger>.05&&rings>2){
            method.setIntakePower(.5);
        }
        else if(gamepad1.left_trigger>.05&&rings>2){
            method.setIntakePower(-.5);
        }
        else{
            method.setIntakePower(0);
        }
    }
    public void claw(){
        if((gamepad1.b && !isBPressed)){
            isBPressed = true;
            if (!clawClosed) {
                method.controlClawServo(.25);//closing claw
                isRunning = true;
                method.runtime.reset();

            }
            else{
                method.controlArmServo(1);//moving arm down
                isRunning = true;
                method.runtime.reset();
            }
        }
        if(!gamepad1.b){
            isBPressed = false;
        }
        if (isRunning){
            if (!clawClosed){
                if (method.runtime.seconds() > .5) {
                    method.controlArmServo(.25);//move arm up
                    clawClosed = true;
                    isRunning = false;
                }

            }
            else{
                if (method.runtime.seconds() > .5) {
                    method.controlClawServo(.7);//opening claw
                    clawClosed = false;
                    isRunning = false;
                }
            }
        }
    }
    public void ringIn(){
        if(!RingIn&&method.robot.distance.getDistance(DistanceUnit.CM)<5){
            RingIn = true;
            intakingInitially = intakingRing;
        }
        if(RingIn&&method.robot.distance.getDistance(DistanceUnit.CM)>5){
            RingIn= false;
            if(intakingRing&&intakingInitially) {
                rings++;
            }
            else if(!intakingRing&&!intakingInitially){
                rings--;
            }
        }
    }
    public void resetAngle() {
        if (gamepad1.y) {
            method.resetAngle = method.getHeading() + method.resetAngle;
            method.resetAngle2 = method.getHeading2() +method.resetAngle2;
            method.robot.encoders.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            method.robot.intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    //not in use
    public void updateShootingParameters(){
        method.updateShootingParameters();
    }
    public void goToPosition(){
        if(gamepad1.left_trigger>.1){
            method.goToPosition(0, method.currentXPosition, method.currentYPosition, 36, 72);
        }
    }
    public void updatePosition(){
        if (gamepad1.dpad_left){
            method.stopAndResetEncoders();
            method.currentXPosition = 9;
            method.currentYPosition = 9;
            method.resetAngle = method.getHeading() + method.resetAngle;
        }

        double rotation = (method.robot.backLeftMotor.getCurrentPosition() - method.robot.frontRightMotor.getCurrentPosition()) / 2.0;

        double currentY = method.robot.backLeftMotor.getCurrentPosition()*Math.sqrt(2)-rotation;
        double deltaY1 = currentY-previousY;
        //telemetry.addData("Delta y1", deltaY1);

        double currentX = method.robot.backRightMotor.getCurrentPosition()*Math.sqrt(2)+rotation;
        double deltaX1 = currentX-previousX;
        //telemetry.addData("Delta x1", deltaX1);

        double thetaX = Math.PI/4;
        double thetaY = 3*Math.PI/4;

        //changing from a [+] with | being y and -- being x to an [X] with \ being y and / being x (forward is forward)
        //double rotatedTheta = theta + (Math.PI / 4);
        double gyroAngle = method.getHeading() * Math.PI / 180; //Converts gyroAngle into radians

        double calculationAngleX =  thetaX-gyroAngle;
        double calculationAngleY =  thetaY-gyroAngle;
        //telemetry.addData("angleX", calculationAngleX);
        //telemetry.addData("angleY", calculationAngleY);

        double deltaY2 = Math.sin(calculationAngleY) * deltaY1;
        if(Math.sin(calculationAngleX) * deltaX1>Math.sin(calculationAngleY) * deltaY1){
            deltaY2 = Math.sin(calculationAngleX) * deltaX1;
        }
        //    telemetry.addLine("1");
        if((Math.sin(calculationAngleY) * deltaY1>0&&Math.sin(calculationAngleX) * deltaX1<0)||(Math.sin(calculationAngleY) * deltaY1<0&&Math.sin(calculationAngleX) * deltaX1>0)) {
            deltaY2 = Math.sin(calculationAngleY) * deltaY1 + Math.sin(calculationAngleX) * deltaX1;
            //    telemetry.addLine("2");
        }
        double deltaX2 = Math.cos(calculationAngleX) * deltaX1;
        if(Math.cos(calculationAngleY) * deltaY1>Math.cos(calculationAngleX) * deltaX1){
            //    deltaX2 = Math.cos(calculationAngleY) * deltaY1;
        }
        if((Math.cos(calculationAngleY) * deltaY1>0&&Math.cos(calculationAngleX) * deltaX1<0)||(Math.cos(calculationAngleY) * deltaY1<0&&Math.cos(calculationAngleX) * deltaX1>0)) {
            deltaX2 = Math.cos(calculationAngleY) * deltaY1 + Math.cos(calculationAngleX) * deltaX1;
        }
        //telemetry.addData("Delta y2", deltaY2);

        //telemetry.addData("Delta y2", deltaY2);

        double DistY = (method.wheelCircumference) * ((deltaY2) / method.countsPerRotation);
        //telemetry.addData("Delta y", DistY);

        double DistX = (method.wheelCircumference) * ((deltaX2) / method.countsPerRotation);
        //telemetry.addData("Delta x", DistX);

        if(Math.abs(deltaY2)>30 || Math.abs(deltaX2)>30) {
            method.currentYPosition += DistY;
            previousY = currentY;

            if(method.currentYPosition<9){
                method.currentYPosition = 9;
            }

            else if (method.currentYPosition>132.5){
                method.currentYPosition = 132.5;
            }

            method.currentXPosition += DistX;
            previousX = currentX;

            if(method.currentXPosition<9){
                method.currentXPosition = 9;
            }

            else if (method.currentXPosition>86){
                method.currentXPosition = 86;
            }
        }
    }
    public void updatePosition2(){
        if (gamepad1.dpad_left){
            method.stopAndResetEncoders();
            method.currentXPosition = 9;
            method.currentYPosition = 9;
            method.resetAngle = method.getHeading() + method.resetAngle;
        }

        double rotation = (method.robot.backLeftMotor.getCurrentPosition() - method.robot.frontRightMotor.getCurrentPosition()) / 2.0;

        double currentY = method.robot.backLeftMotor.getCurrentPosition()*Math.sqrt(2)-rotation;
        double deltaY1 = currentY-previousY;
        //telemetry.addData("Delta y1", deltaY1);

        double currentX = method.robot.backRightMotor.getCurrentPosition()*Math.sqrt(2)+rotation;
        double deltaX1 = currentX-previousX;
        //telemetry.addData("Delta x1", deltaX1);

        double thetaX = Math.PI/4;
        double thetaY = 3*Math.PI/4;

        //changing from a [+] with | being y and -- being x to an [X] with \ being y and / being x (forward is forward)
        //double rotatedTheta = theta + (Math.PI / 4);
        double gyroAngle = method.getHeading() * Math.PI / 180; //Converts gyroAngle into radians

        double calculationAngleX =  thetaX-gyroAngle;
        double calculationAngleY =  thetaY-gyroAngle;
        //telemetry.addData("angleX", calculationAngleX);
        //telemetry.addData("angleY", calculationAngleY);

        double deltaY2 = Math.sin(calculationAngleY) * deltaY1;
        if(Math.sin(calculationAngleX) * deltaX1>Math.sin(calculationAngleY) * deltaY1){
            deltaY2 = Math.sin(calculationAngleX) * deltaX1;
        }
        //    telemetry.addLine("1");
        if((Math.sin(calculationAngleY) * deltaY1>0&&Math.sin(calculationAngleX) * deltaX1<0)||(Math.sin(calculationAngleY) * deltaY1<0&&Math.sin(calculationAngleX) * deltaX1>0)) {
            deltaY2 = Math.sin(calculationAngleY) * deltaY1 + Math.sin(calculationAngleX) * deltaX1;
            //    telemetry.addLine("2");
        }
        double deltaX2 = Math.cos(calculationAngleX) * deltaX1;
        if(Math.cos(calculationAngleY) * deltaY1>Math.cos(calculationAngleX) * deltaX1){
            //    deltaX2 = Math.cos(calculationAngleY) * deltaY1;
        }
        if((Math.cos(calculationAngleY) * deltaY1>0&&Math.cos(calculationAngleX) * deltaX1<0)||(Math.cos(calculationAngleY) * deltaY1<0&&Math.cos(calculationAngleX) * deltaX1>0)) {
            deltaX2 = Math.cos(calculationAngleY) * deltaY1 + Math.cos(calculationAngleX) * deltaX1;
        }
        //telemetry.addData("Delta y2", deltaY2);

        //telemetry.addData("Delta y2", deltaY2);

        double DistY = (method.wheelCircumference) * ((deltaY2) / method.countsPerRotation);
        //telemetry.addData("Delta y", DistY);

        double DistX = (method.wheelCircumference) * ((deltaX2) / method.countsPerRotation);
        //telemetry.addData("Delta x", DistX);

        if(Math.abs(deltaY2)>30 || Math.abs(deltaX2)>30) {
            method.currentYPosition += DistY;
            previousY = currentY;

            if(method.currentYPosition<9){
                method.currentYPosition = 9;
            }

            else if (method.currentYPosition>132.5){
                method.currentYPosition = 132.5;
            }

            method.currentXPosition += DistX;
            previousX = currentX;

            if(method.currentXPosition<9){
                method.currentXPosition = 9;
            }

            else if (method.currentXPosition>86){
                method.currentXPosition = 86;
            }
        }
    }
    public void shoot(){
        if(gamepad1.x) {
            method.shootRings(3);
            rings=0;
        }
    }
    public void shootingAutomatically(){
        if (rings==3&&method.runtime3.seconds()<95&&method.currentYPosition<84){
            method.shoot(method.shootingAngle, method.shooterPower);
        }
    }

}
