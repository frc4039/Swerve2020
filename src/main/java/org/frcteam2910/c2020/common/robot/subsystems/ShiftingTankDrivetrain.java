package org.frcteam2910.c2020.common.robot.subsystems;

@Deprecated
public abstract class ShiftingTankDrivetrain extends TankDrivetrain {

	public ShiftingTankDrivetrain(double trackWidth) {
		super(trackWidth);
	}

	public abstract boolean inHighGear();

	public abstract void setHighGear(boolean highGear);
}
