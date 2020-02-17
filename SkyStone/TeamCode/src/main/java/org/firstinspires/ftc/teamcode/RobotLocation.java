package org.firstinspires.ftc.teamcode;

class RobotLocation {
    private double[] pos = {0,0};
    private double rot = 0;
    public RobotLocation(){

    }

    public double getRot() {
        return rot;
    }

    public double[] getPos() {
        return pos;
    }

    public void setPos(double[] pos) {
        this.pos = pos;
    }

    public void setRot(double rot) {
        this.rot = rot;
    }

    public void incrementOffsets(double[] dPos, double dTheta) {
        double[] rotatedDPos = {dPos[0]*Math.cos(this.rot)-dPos[1]*Math.sin(this.rot),dPos[0]*Math.sin(this.rot)+dPos[1]*Math.cos(this.rot)};
        this.rot += dTheta;
        this.pos[0] += rotatedDPos[0]; this.pos[1] += rotatedDPos[1];
    }

    public String toString() {
        return this.getInfo();
    }

    public String getInfo() {
        return "Position: (" + Math.round(pos[0]*1000.0)/1000.0 + ", " + Math.round(pos[1]*1000.0)/1000.0 + "), Angle: " + Math.round(rot*1000.0)/1000.0;
    }
}
