// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Util {

    public static Translation3d getNormalTranslation(Translation3d vec){
        return(new Translation3d(vec.getX()/vec.getNorm(), vec.getY()/vec.getNorm(), vec.getZ()/vec.getNorm()));
    }

    public static class PIDConstants{
        public double mKP;
        public double mKI;
        public double mKD;
        public double mKV;
        public double mKS;
        public double mContinuousInputMax;
        public double mContinuousInputMin;
        public boolean mIsContinuousInput;


        public PIDConstants(double kP, double kI, double kD, double kV, double kS, double continuousInputMax, double continuousInputMin){
            mKP = kP;
            mKI = kI;
            mKD = kD;
            mKV = kV;
            mKS = kS;
            mContinuousInputMax = continuousInputMax;
            mContinuousInputMin = continuousInputMin;
            mIsContinuousInput = true;
        }

        public PIDConstants(double kP, double kI, double kD, double kS, double kV){
            new PIDConstants(kP, kI, kD, kS, kV,0,0);
            mIsContinuousInput = false;
        }

        public PIDConstants(double kP, double kI, double kD){
            new PIDConstants(kP, kI, kD, 0, 0,0,0);
            mIsContinuousInput = false;
        }
        

        public PIDController toWPIController(){
            PIDController pid = new PIDController(mKP, mKI, mKD);
            if(mIsContinuousInput){pid.enableContinuousInput(mContinuousInputMin, mContinuousInputMax);}
            return pid;
        }

        public TalonFXConfiguration toTalonConfiguration(){
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.Slot0.kP = mKP;//0.2
            config.Slot0.kI = mKI;
            config.Slot0.kD = mKD;//0.05
            config.Slot0.kV = mKV;
            config.Slot0.kS = mKS;
            config.ClosedLoopGeneral.ContinuousWrap = mIsContinuousInput;
            return config;
        }
    }
    public enum POV{
        DPADUP(0),
        DPADRIGHT(90),
        DPADDOWN(180),
        DPADLEFT(270);

        public final int mAngle;
        private POV(int angle){
            mAngle = angle;
        }
    }
    public static class TBHController{
        private double mCurOutput = 0;
        private double mPrevError = 0;
        private double mSetPoint = 0;
        private double mTBHConstant;

        public final double mF;
        private final double mMaxOutput;
        private final double mTolerance;

        public TBHController(double f, double maxOutput,double tolerance){
            mF = f;
            mMaxOutput = maxOutput;
            mTolerance = tolerance;
        }

        public TBHController(double f, double maxOutput){
            mF = f;
            mMaxOutput = maxOutput;
            mTolerance = 0;
        }
        public double calculate(double measurement){
            double error = mSetPoint-measurement;
            mCurOutput += error*mF;
            MathUtil.clamp(mCurOutput, -1, 1);
            if (error>0 != mPrevError>0){
                mCurOutput = (mCurOutput + mTBHConstant)/2;
                mTBHConstant = mCurOutput;
                mPrevError = error;
            }
            return mCurOutput;
        }
        public void setSpNoSpinUp(double f){
            mSetPoint = f;
        }
        public void spinUp(double targetOutput){
            if(mSetPoint > targetOutput){
                mPrevError = 1;
            }else if (mSetPoint < targetOutput){
                mPrevError = -1;
            }
            mTBHConstant = 2*(targetOutput/mMaxOutput)-1;
            mSetPoint = targetOutput;
        }
        public boolean atSetpoint(){
            return Math.abs(mCurOutput-mSetPoint) <= mTolerance;
        }

        public void writeDebugData() {
            SmartDashboard.putNumber("mF Constant TBH Controller", mF);
            SmartDashboard.putNumber("Previous Error TBH Controller", mPrevError);
            SmartDashboard.putBoolean("TBH Controller", atSetpoint());
        }

        public double getCurOutput(){return mCurOutput;}
        public double getPrevError(){return mPrevError;}
        public double getSetpoint(){return mSetPoint;}
        public double getTBHConstant(){return mTBHConstant;}
    }
}
