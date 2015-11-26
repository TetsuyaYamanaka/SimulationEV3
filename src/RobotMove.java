import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

class Parameter{ //���{�b�g�̃p�����[�^
	double v; //�ԑ̑��x(m/s)
	double omega; //�ԑ̉�]�p�x(rad)
	double[] sisei = new double[2]; //�ԑ̎p���p
	double d_omega; //�T���v�����O�^�C���̎ԑ̉�]�p�x
	double a; //�T���v�����O�^�C���̑��s����(m)
	Point2D.Double location = new Point2D.Double(); //���{�b�g�̈ʒu
}

class RobotMove extends TimerTask{
	final int timeMax = 5;
	final double dtor = Math.PI/180; //deg��rad
	final double tread = 0.122; //�g���b�h(m)
	final double hankei = 0.15; //�ԗ֔��a(m)
	final long delay = 0;
	final long period = 10;
	final double mstos = 0.001; //msec��sec
	int i;
	double dt;
	Point2D.Double observe; //�ϑ��_���W
	long time0,time;
	Parameter robo = new Parameter(); //���{�b�g�̃p�����[�^�\����
	MakeGridmap gm = new MakeGridmap(); //�O���b�h�}�b�v�C���X�^���X
	SensorMearsure sm; //�Z���T�C���X�^���X�i�Q�Ɓj
	
	RobotMove(){ //�ԑ̑��x�A��]�p�x�A�����p����ݒ�
		robo.v = 0.04;
		robo.omega = 0*dtor;
		robo.sisei[0] = 0;
		time0 = System.nanoTime();
	}
	
	public void run(){
		time = TimeUnit.MILLISECONDS.convert(System.nanoTime()-time0, TimeUnit.NANOSECONDS);
		if(time>=(long)timeMax*1000){
			gm.PrintGrid();
			this.cancel();
			System.exit(0);
		}
		Odometry();
		gm.UpdateGrid(robo.location,1);
		/*if(sm.flag == 1){
			gm.UpdateGrid(sm.obserW, 2);
		}*/
		System.out.println("robot: " + time);
	}
	
	public void SetInstance(SensorMearsure sensor){ //�Z���T�C���X�^���X�ւ̃|�C���^���Z�b�g
		sm = sensor;
	}
	
	public void Odometry(){ //�I�h���g��
		robo.a = robo.v*(period*mstos);
		robo.sisei[1] = (robo.omega/timeMax)*(period*mstos) + robo.sisei[0];
		robo.location.setLocation(robo.a*Math.cos(robo.sisei[1]) + robo.location.x,robo.a*Math.sin(robo.sisei[1]) + robo.location.y); //���{�b�g�̌��݈ʒu�𐄒�
		robo.d_omega = robo.sisei[1] - robo.sisei[0];
		robo.sisei[0] = robo.sisei[1];
	}
}
