import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

public class SensorMearsure {
	final long delay = 0;
	final long period = 50;
	
	Parameter robo = new Parameter();
	Point2D.Double obst_vir = new Point2D.Double(0.4,0.2); //障害物座標（ワールド系）を設定
	Point2D.Double obser = new Point2D.Double(),obser2 = new Point2D.Double(); //ロボットから見た観測点
	Point2D.Double obserW = new Point2D.Double(),obserW2 = new Point2D.Double(); //ワールド座標系での観測点
	double[] len = new double[2];
	double d_theta,theta;
	double theta2;
	int timemax = 5;
	double sin_theta;
	double cos_d_theta;
	long time0,time;
	
	public Point2D.Double GetObserve(){ //観測点の座標を取得
		return obserW;
	}
	
	public void Observe(){ //距離データから障害物位置の推定
		len[1] = obst_vir.distance(robo.location);
		cos_d_theta = (len[1]*len[1] + len[0]*len[0] - robo.a*robo.a)/(2*len[1]*len[0]);
		d_theta = Math.acos(cos_d_theta); //逆三角関数によりΔθを求める
		if(robo.a != 0 && robo.d_omega == 0){ //直進
			sin_theta = len[0]*Math.sin(d_theta)/robo.a;
			theta = Math.asin(sin_theta);//逆三角関数によりθを求める
			theta2 = -theta;
			
			obser.x = len[1]*Math.sin(theta); obser.y = len[1]*Math.cos(theta);
			obser2.x = len[1]*Math.sin(theta2); obser2.y = len[1]*Math.cos(theta2);
			
			obserW.x = obser.x*Math.sin(robo.sisei[1]) + obser.y*Math.cos(robo.sisei[1]) + robo.location.x;
			obserW.y = -obser.x*Math.cos(robo.sisei[1]) + obser.y*Math.sin(robo.sisei[1]) + robo.location.y;
			obserW2.x = obser2.x*Math.sin(robo.sisei[1]) + obser2.y*Math.cos(robo.sisei[1]) + robo.location.x;
			obserW2.y = -obser2.x*Math.cos(robo.sisei[1]) + obser2.y*Math.sin(robo.sisei[1]) + robo.location.y;
		}
		else if(robo.a == 0 && robo.d_omega != 0){ //その場で回転
			obser.x = len[1]*Math.cos(Math.PI/2 + robo.d_omega);
			obser.y = len[1]*Math.sin(Math.PI/2 + robo.d_omega);
			
			obserW.x = obser.x*Math.sin(robo.sisei[1]+robo.d_omega) + obser.y*Math.cos(robo.sisei[1]+robo.d_omega) + robo.location.x;
			obserW.y = -obser.x*Math.cos(robo.sisei[1]+robo.d_omega) + obser.y*Math.sin(robo.sisei[1]+robo.d_omega) + robo.location.y;
		}
		else if(robo.a != 0 && robo.d_omega != 0){ //旋回
			theta = d_theta + Math.abs(robo.d_omega);
			theta2 = -theta;
			if(robo.d_omega < 0){
				obser.x = len[1]*Math.sin(theta); obser.y = len[1]*Math.cos(theta);
				
				obserW.x = obser.x*Math.sin(robo.sisei[1]-d_theta-robo.d_omega) + obser.y*Math.cos(robo.sisei[1]-d_theta-robo.d_omega) + robo.location.x;
				obserW.y = -obser.x*Math.cos(robo.sisei[1]-d_theta-robo.d_omega) + obser.y*Math.sin(robo.sisei[1]-d_theta-robo.d_omega) + robo.location.y;
			}
			else{
				obser.x = -len[1]*Math.sin(theta); obser.y = len[1]*Math.cos(theta);
				
				obserW.x = obser.x*Math.sin(robo.sisei[1]+d_theta+robo.d_omega) + obser.y*Math.cos(robo.sisei[1]+d_theta+robo.d_omega) + robo.location.x;
				obserW.y = -obser.x*Math.cos(robo.sisei[1]+d_theta+robo.d_omega) + obser.y*Math.sin(robo.sisei[1]+d_theta+robo.d_omega) + robo.location.y;
			}
		}
	}
	
	public void TaskRun(RobotMove rm){
		robo = rm.GetRobotParameter();
		time0 = System.nanoTime();
		
		TimerTask task = new TimerTask(){
			public void run(){
				time = TimeUnit.MILLISECONDS.convert(System.nanoTime()-time0, TimeUnit.NANOSECONDS);
				
				if(time>5000){
					this.cancel();
				}
				Observe();
				System.out.println("sensor: " + time);
			}
		};
		Timer timer = new Timer();
		timer.scheduleAtFixedRate(task, delay, period);
	}
}
