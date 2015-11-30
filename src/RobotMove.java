import java.awt.geom.Point2D;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;
import java.io.*;

class Parameter{ //���{�b�g�̃p�����[�^
	double v; //�ԑ̑��x(m/s)
	double omega; //�ԑ̉�]�p�x(rad)
	double[] sisei = new double[2]; //�ԑ̎p���p
	double d_omega; //�T���v�����O�^�C���̎ԑ̉�]�p�x
	double a; //�T���v�����O�^�C���̑��s����(m)
	double sumA = 0;
	Point2D.Double location = new Point2D.Double(); //���{�b�g�̈ʒu
}

class CalcObst{ //��Q�����W�v�Z�Ɋւ���p�����[�^
	Point2D.Double obser = new Point2D.Double(),obser2 = new Point2D.Double(); //���{�b�g���猩���ϑ��_
	Point2D.Double obserW = new Point2D.Double(),obserW2 = new Point2D.Double(); //���[���h���W�n�ł̊ϑ��_
	double d_theta,theta;
	double theta2;
	double sin_theta;
	double cos_d_theta;
}

class RobotMove extends TimerTask{
	final int timeMax = 5;
	final double dtor = Math.PI/180; //deg��rad
	final double tread = 0.122; //�g���b�h(m)
	final double hankei = 0.15; //�ԗ֔��a(m)
	final long period = 10;
	final long periodSensor = 50;
	final double mstos = 0.001; //msec��sec
	int i;
	double dt;
	long time0,time;
	Parameter robo = new Parameter(); //���{�b�g�̃p�����[�^�\����
	CalcObst co = new CalcObst();
	MakeGridmap gm = new MakeGridmap(); //�O���b�h�}�b�v�C���X�^���X
	SensorMearsure sm; //�Z���T�C���X�^���X�i�Q�Ɓj
	
	RobotMove(long startTime){ //�ԑ̑��x�A��]�p�x�A�����p����ݒ�
		robo.v = 0.04;
		robo.omega = 0*dtor;
		robo.sisei[0] = 0;
		time0 = startTime;
	}
	
	public void run(){
		time = TimeUnit.MILLISECONDS.convert(System.nanoTime()-time0, TimeUnit.NANOSECONDS);
		if(time>=(long)timeMax*1000){
			gm.PrintGrid();
			this.cancel();
		}
		Odometry();
		if(sm.flag==1){
			Observe();
			gm.GaussianUpdate(co.obserW);
			gm.GaussianUpdate(co.obserW2);
			robo.sumA = 0;
		}
		gm.UpdateGrid(robo.location,1);
		//System.out.println("robot: " + time);
	}
	
	public void SetInstance(SensorMearsure sensor){ //�Z���T�C���X�^���X�ւ̎Q�Ƃ��Z�b�g
		sm = sensor;
	}
	
	public void Odometry(){ //�I�h���g��
		robo.a = robo.v*(period*mstos);
		robo.sumA += robo.a;
		robo.sisei[1] = (robo.omega/timeMax)*(period*mstos) + robo.sisei[0];
		robo.location.setLocation(robo.a*Math.cos(robo.sisei[1]) + robo.location.x,robo.a*Math.sin(robo.sisei[1]) + robo.location.y); //���{�b�g�̌��݈ʒu�𐄒�
		robo.d_omega = robo.sisei[1] - robo.sisei[0];
		robo.sisei[0] = robo.sisei[1];
	}
	
	public void Observe(){ //�����f�[�^�����Q���ʒu�̐���
		co.cos_d_theta = (sm.len[1]*sm.len[1] + sm.len[0]*sm.len[0] - robo.sumA*robo.sumA)/(2*sm.len[1]*sm.len[0]);
		co.d_theta = Math.acos(co.cos_d_theta); //�t�O�p�֐��ɂ�胢�Ƃ����߂�
		if(robo.sumA != 0 && robo.d_omega == 0){ //���i
			co.sin_theta = sm.len[0]*Math.sin(co.d_theta)/robo.sumA;
			co.theta = Math.asin(co.sin_theta);//�t�O�p�֐��ɂ��Ƃ����߂�
			co.theta2 = -co.theta;
			
			co.obser.x = sm.len[1]*Math.sin(co.theta); co.obser.y = sm.len[1]*Math.cos(co.theta);
			co.obser2.x = sm.len[1]*Math.sin(co.theta2); co.obser2.y = sm.len[1]*Math.cos(co.theta2);
			
			co.obserW.x = co.obser.x*Math.sin(robo.sisei[1]) + co.obser.y*Math.cos(robo.sisei[1]) + robo.location.x;
			co.obserW.y = -co.obser.x*Math.cos(robo.sisei[1]) + co.obser.y*Math.sin(robo.sisei[1]) + robo.location.y;
			co.obserW2.x = co.obser2.x*Math.sin(robo.sisei[1]) + co.obser2.y*Math.cos(robo.sisei[1]) + robo.location.x;
			co.obserW2.y = -co.obser2.x*Math.cos(robo.sisei[1]) + co.obser2.y*Math.sin(robo.sisei[1]) + robo.location.y;
		}
		else if(robo.sumA == 0 && robo.d_omega != 0){ //���̏�ŉ�]
			co.obser.x = sm.len[1]*Math.cos(Math.PI/2 + robo.d_omega);
			co.obser.y = sm.len[1]*Math.sin(Math.PI/2 + robo.d_omega);
			
			co.obserW.x = co.obser.x*Math.sin(robo.sisei[1]+robo.d_omega) + co.obser.y*Math.cos(robo.sisei[1]+robo.d_omega) + robo.location.x;
			co.obserW.y = -co.obser.x*Math.cos(robo.sisei[1]+robo.d_omega) + co.obser.y*Math.sin(robo.sisei[1]+robo.d_omega) + robo.location.y;
		}
		else if(robo.sumA != 0 && robo.d_omega != 0){ //����
			co.theta = co.d_theta + Math.abs(robo.d_omega);
			co.theta2 = -co.theta;
			if(robo.d_omega < 0){
				co.obser.x = sm.len[1]*Math.sin(co.theta); co.obser.y = sm.len[1]*Math.cos(co.theta);
				
				co.obserW.x = co.obser.x*Math.sin(robo.sisei[1]-co.d_theta-robo.d_omega) + co.obser.y*Math.cos(robo.sisei[1]-co.d_theta-robo.d_omega) + robo.location.x;
				co.obserW.y = -co.obser.x*Math.cos(robo.sisei[1]-co.d_theta-robo.d_omega) + co.obser.y*Math.sin(robo.sisei[1]-co.d_theta-robo.d_omega) + robo.location.y;
			}
			else{
				co.obser.x = -sm.len[1]*Math.sin(co.theta); co.obser.y = sm.len[1]*Math.cos(co.theta);
				
				co.obserW.x = co.obser.x*Math.sin(robo.sisei[1]+co.d_theta+robo.d_omega) + co.obser.y*Math.cos(robo.sisei[1]+co.d_theta+robo.d_omega) + robo.location.x;
				co.obserW.y = -co.obser.x*Math.cos(robo.sisei[1]+co.d_theta+robo.d_omega) + co.obser.y*Math.sin(robo.sisei[1]+co.d_theta+robo.d_omega) + robo.location.y;
			}
		}
	}
}
