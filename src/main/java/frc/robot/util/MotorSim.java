package frc.robot.util;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class MotorSim {
    double m_CurrentPosition = 0;
    double m_CurrentVelocity = 0;
    boolean m_Inverted = false;
    MotorSim m_OtherMotor;
    double m_Id;

    float m_SoftLimitF;
    float m_SoftLimitR;
    boolean m_SoftLimitFEnabled = false;
    boolean m_SoftLimitREnabled = false;
    Mode m_Mode;
    double m_P;
    double m_I;
    double m_D;
    double m_FF;
    double m_DesiredPosition;
    





     public void setCurrentVelocity(double v) {
        this.m_CurrentVelocity = v;
    }

    public void setCurrentPosition(double x) {
        this.m_CurrentPosition = x;
    }

    public double getPosition() {
        return this.m_CurrentPosition;
    }
    public double getVelocity() {
        return this.m_CurrentVelocity;
    }

    public void setDesiredPosition(double wantedPosition) {
       this.m_DesiredPosition = wantedPosition;
    }


   public MotorSim(int ID, MotorType type, Mode mode) {
    this.m_Id = ID;
    this.m_Mode = mode;
    

   }

   public void update(double dt) {
    if (!SoftLimitHit()) { 
    if (this.m_OtherMotor != null) {
        this.m_CurrentPosition = this.m_OtherMotor.m_CurrentPosition;
    } else {
        if (m_Mode == Mode.PID) {
            double error = m_DesiredPosition-m_CurrentPosition;
            m_CurrentVelocity = m_P*error;
        } }
        this.m_CurrentPosition += dt*m_CurrentVelocity;
   }
}


  public enum SoftLimitDirection {
    kForward,
    kReverse
  }

  public enum Mode {
    MANUAL,
    PID
  }


  public void setInverted(boolean isInverted) {
  }

  public void setSmartCurrentLimit(int limit) {
  }

  public void setVelocityConversionFactor(double vFactor) {
  }

  public void setPositionConversionFactor(double pFactor) {

  }



public void follow(MotorSim leader) {
    this.follow(leader, true);
}


public void follow(MotorSim leader, boolean x) {
    this.m_OtherMotor = leader;
}

public void set(double speed) {
    setCurrentVelocity(speed);
}

public void setPID(double P, double I, double D) {
    this.m_P = P;
    this.m_I = I;
    this.m_D = D;
}

public void setP(double P) {
    this.m_P = P;
}

public void setI(double I) {
    this.m_I = I;
}

public void setD(double D) {
    this.m_D = D;
}

public void setFF(double FF) {
    this.m_FF = FF;
}




public void setSoftLimit(SoftLimitDirection direction, float limit) {
    if (direction == SoftLimitDirection.kForward) this.m_SoftLimitF = limit;
    else  m_SoftLimitR = limit;
}


public void enableSoftLimit(SoftLimitDirection direction, boolean x) {
   
    if (direction == SoftLimitDirection.kForward) this.m_SoftLimitFEnabled = x;
        else this.m_SoftLimitREnabled = x;
}

public boolean SoftLimitHit() {
return ((this.m_CurrentPosition > this.m_SoftLimitF && this.m_SoftLimitFEnabled && this.m_CurrentVelocity>0) || 
        (this.m_CurrentPosition < this.m_SoftLimitR && this.m_SoftLimitREnabled && this.m_CurrentVelocity<0));
}
















}
