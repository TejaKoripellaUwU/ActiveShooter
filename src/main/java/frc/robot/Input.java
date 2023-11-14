// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class Input {

    private static final XboxController mController = new XboxController(1);
    public static double getLeftJoyY(){return mController.getLeftY();}
    public static double getRightJoyY(){return mController.getRightY();}
    public static boolean getXPressed(){return mController.getXButton();}
    public static boolean getYPressed(){return mController.getYButton();}
    public static boolean getAPressed(){return mController.getAButton();}
    public static boolean getBPressed(){return mController.getBButton();}


}
