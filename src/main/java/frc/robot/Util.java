// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class Util {
    public static Translation3d getNormalTranslation(Translation3d vec){
        return(new Translation3d(vec.getX()/vec.getNorm(), vec.getY()/vec.getNorm(), vec.getZ()/vec.getNorm()));
    }
    public static class TBHController{
        private double mCurOutput = 0;
        private double mPrevError = 0;
        private double mSetPoint = 0;

        private double mF;
        private double mTBHConstant;
        private double mMaxOutput;
        private double mTolerance;
        private double mLastMeasurement = 0;

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
        public void setF(double f){
            mF = f;
        }
        public void setTolerance(double tolerance){
            mTolerance = tolerance;
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
            return Math.abs(mLastMeasurement-mSetPoint) <= mTolerance;
        }
    }
}
