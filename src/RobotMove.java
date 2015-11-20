import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

class Parameter{ //ロボットのパラメータ
	double v; //車体速度(m/s)
	double omega; //車体回転角度(rad)
	double[] sisei = new double[2]; //車体姿勢角
	double d_omega; //サンプリングタイムの車体回転角度
	double a; //サンプリングタイムの走行距離(m)
	Point2D.Double location = new Point2D.Double(); //ロボットの位置
}

public class RobotMove {
	final int timemax = 5;
	final double dtor = Math.PI/180; //deg→rad
	final double tread = 0.122; //トレッド(m)
	final double hankei = 0.15; //車輪半径(m)
	final long delay = 0;
	final long period = 10;
	final double mstos = 0.001; //msec→sec
	int i;
	long time,time0;
	double dt;
	Point2D.Double observe; //観測点座標
	Parameter robo = new Parameter(); //ロボットのパラメータ構造体
	
	MakeGridmap gm = new MakeGridmap();
	
	RobotMove(){ //車体速度、回転角度、初期姿勢を設定
		robo.v = 0.04;
		robo.omega = 0*dtor;
		robo.sisei[0] = 0;
	}
	
	public void GetGridmap(MakeGridmap map){ //グリッドマップを取得
		map = gm;
	}
	
	public Parameter GetRobotParameter(){ //ロボットの現在のパラメータを取得
		return robo;
	}
	
	public void Odometry(){ //オドメトリ
		robo.a = robo.v*(period*mstos);
		robo.sisei[1] = (robo.omega/timemax)*period + robo.sisei[0];
		robo.location.setLocation(robo.a*Math.cos(robo.sisei[1]) + robo.location.x,robo.a*Math.sin(robo.sisei[1]) + robo.location.y); //ロボットの現在位置を推定
		robo.d_omega = robo.sisei[1] - robo.sisei[0];
		robo.sisei[0] = robo.sisei[1];
	}
	
	public void UpdateMaptoFloor(MakeGridmap gm){ //グリッドの状態を床確定に更新
		gm.UpdateGrid(robo.location, 1);
	}
	
	public void UpdateMaptoObst(SensorMearsure sm,MakeGridmap gm){ //グリッドの状態を障害物確定に更新
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
