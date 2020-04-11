package org.firstinspires.ftc.teamcode;

public class AngleSystem{
    double axesZeroAngle; //measured as a regular mathematically correct angle in radians, with positive first axis as 0 and positive second as Math.PI/2
    boolean direction; //true is from positive first to positive second, false is from postive second to positive first;
    boolean isDegrees = false;//true measures in degrees instead of radians
    public static final AngleSystem GLOBAL = new AngleSystem(0,true);

    public AngleSystem(double globalZero, boolean dir, boolean deg){
        initSelf(globalZero, dir, deg);
    }

    public AngleSystem(double globalZero, boolean dir){
        initSelf(globalZero,dir,false);
    }

    public void initSelf(double globalZero, boolean dir, boolean deg){
        axesZeroAngle = globalZero;
        direction = dir;
        isDegrees = deg;
    }

    //always returns in radians because global is radians
    public double toGlobal(double inAngle){
        return (axesZeroAngle + inAngle*(direction ? 1 : -1)*(isDegrees?Math.PI/180:1) + Math.PI*4) % (Math.PI*2);
    }

    public double fromGlobal(double globalAngle){
        return (globalAngle - axesZeroAngle)*(direction ? 1 : -1)*(isDegrees?180/Math.PI:1);

    }

    public double convertTo(double inAngle, AngleSystem target){
        return target.fromGlobal(toGlobal(inAngle));
    }


    public double convertTo(double inAngle, AngleSystem target,boolean returnRads){
        double result = target.fromGlobal(toGlobal(inAngle));
        if (target.isDegrees)
        {
            if (!returnRads)
                return result;
            else
                return result/180*Math.PI;
        }
        else{
            if (returnRads)
                return result;
            else
                return result/Math.PI*180;
        }
    }
}
