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

public class RobotMove {
	final int timemax = 5;
	final double dtor = Math.PI/180; //deg��rad
	final double tread = 0.122; //�g���b�h(m)
	final double hankei = 0.15; //�ԗ֔��a(m)
	final long delay = 0;
	final long period = 10;
	final double mstos = 0.001; //msec��sec
	int i;
	long time,time0;
	double dt;
	Point2D.Double observe; //�ϑ��_���W
	Parameter robo = new Parameter(); //���{�b�g�̃p�����[�^�\����
	
	MakeGridmap gm = new MakeGridmap();
	
	RobotMove(){ //�ԑ̑��x�A��]�p�x�A�����p����ݒ�
		robo.v = 0.04;
		robo.omega = 0*dtor;
		robo.sisei[0] = 0;
	}
	
	public void GetGridmap(MakeGridmap map){ //�O���b�h�}�b�v���擾
		map = gm;
	}
	
	public Parameter GetRobotParameter(){ //���{�b�g�̌��݂̃p�����[�^���擾
		return robo;
	}
	
	public void Odometry(){ //�I�h���g��
		robo.a = robo.v*(period*mstos);
		robo.sisei[1] = (robo.omega/timemax)*period + robo.sisei[0];
		robo.location.setLocation(robo.a*Math.cos(robo.sisei[1]) + robo.location.x,robo.a*Math.sin(robo.sisei[1]) + robo.location.y); //���{�b�g�̌��݈ʒu�𐄒�
		robo.d_omega = robo.sisei[1] - robo.sisei[0];
		robo.sisei[0] = robo.sisei[1];
	}
	
	public void UpdateMaptoFloor(MakeGridmap gm){ //�O���b�h�̏�Ԃ����m��ɍX�V
		gm.UpdateGrid(robo.location, 1);
	}
	
	public void UpdateMaptoObst(SensorMearsure sm,MakeGridmap gm){ //�O���b�h�̏�Ԃ���Q���m��ɍX�V
		observe = sm.GetObserve();
		gm.UpdateGrid(observe, 2);
	}
	
	public void TaskRun(){
		time0 = System.nanoTime();
		
		TimerTask task = new TimerTask(){
			public void run(){
				time = TimeUnit.MILLISECONDS.convert(System.nanoTime()-time0, TimeUnit.NANOSECONDS);
				if(time>5000){
					gm.PrintGrid();
					this.cancel();
				}
				Odometry();
				UpdateMaptoFloor(gm);
				System.out.println("robot: "+time);
			}
		};
		Timer timer = new Timer();
		
		timer.scheduleAtFixedRate(task, delay, period);
		
	}
}
