import java.awt.geom.Point2D;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;
import java.io.*;

class Parameter{ //ロボットのパラメータ
	double v; //車体速度(m/s)
	double omega; //車体回転角度(rad)
	double[] sisei = new double[2]; //車体姿勢角
	double d_omega; //サンプリングタイムの車体回転角度
	double a; //サンプリングタイムの走行距離(m)
	double sumA = 0;
	Point2D.Double location = new Point2D.Double(); //ロボットの位置
}

class CalcObst{ //障害物座標計算に関するパラメータ
	Point2D.Double obser = new Point2D.Double(),obser2 = new Point2D.Double(); //ロボットから見た観測点
	Point2D.Double obserW = new Point2D.Double(),obserW2 = new Point2D.Double(); //ワールド座標系での観測点
	double d_theta,theta;
	double theta2;
	double sin_theta;
	double cos_d_theta;
}

class RobotMove extends TimerTask{
	final int timeMax = 5;
	final double dtor = Math.PI/180; //deg→rad
	final double tread = 0.122; //トレッド(m)
	final double hankei = 0.15; //車輪半径(m)
	final long period = 10;
	final long periodSensor = 50;
	final double mstos = 0.001; //msec→sec
	int i;
	double dt;
	long time0,time;
	Parameter robo = new Parameter(); //ロボットのパラメータ構造体
	CalcObst co = new CalcObst();
	MakeGridmap gm = new MakeGridmap(); //グリッドマップインスタンス
	SensorMearsure sm; //センサインスタンス（参照）
	
	RobotMove(long startTime){ //車体速度、回転角度、初期姿勢を設定
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
	
	public void SetInstance(SensorMearsure sensor){ //センサインスタンスへの参照をセット
		sm = sensor;
	}
	
	public void Odometry(){ //オドメトリ
		robo.a = robo.v*(period*mstos);
		robo.sumA += robo.a;
		robo.sisei[1] = (robo.omega/timeMax)*(period*mstos) + robo.sisei[0];
		robo.location.setLocation(robo.a*Math.cos(robo.sisei[1]) + robo.location.x,robo.a*Math.sin(robo.sisei[1]) + robo.location.y); //ロボットの現在位置を推定
		robo.d_omega = robo.sisei[1] - robo.sisei[0];
		robo.sisei[0] = robo.sisei[1];
	}
	
	public void Observe(){ //距離データから障害物位置の推定
		co.cos_d_theta = (sm.len[1]*sm.len[1] + sm.len[0]*sm.len[0] - robo.sumA*robo.sumA)/(2*sm.len[1]*sm.len[0]);
		co.d_theta = Math.acos(co.cos_d_theta); //逆三角関数によりΔθを求める
		if(robo.sumA != 0 && robo.d_omega == 0){ //直進
			co.sin_theta = sm.len[0]*Math.sin(co.d_theta)/robo.sumA;
			co.theta = Math.asin(co.sin_theta);//逆三角関数によりθを求める
			co.theta2 = -co.theta;
			
			co.obser.x = sm.len[1]*Math.sin(co.theta); co.obser.y = sm.len[1]*Math.cos(co.theta);
			co.obser2.x = sm.len[1]*Math.sin(co.theta2); co.obser2.y = sm.len[1]*Math.cos(co.theta2);
			
			co.obserW.x = co.obser.x*Math.sin(robo.sisei[1]) + co.obser.y*Math.cos(robo.sisei[1]) + robo.location.x;
			co.obserW.y = -co.obser.x*Math.cos(robo.sisei[1]) + co.obser.y*Math.sin(robo.sisei[1]) + robo.location.y;
			co.obserW2.x = co.obser2.x*Math.sin(robo.sisei[1]) + co.obser2.y*Math.cos(robo.sisei[1]) + robo.location.x;
			co.obserW2.y = -co.obser2.x*Math.cos(robo.sisei[1]) + co.obser2.y*Math.sin(robo.sisei[1]) + robo.location.y;
		}
		else if(robo.sumA == 0 && robo.d_omega != 0){ //その場で回転
			co.obser.x = sm.len[1]*Math.cos(Math.PI/2 + robo.d_omega);
			co.obser.y = sm.len[1]*Math.sin(Math.PI/2 + robo.d_omega);
			
			co.obserW.x = co.obser.x*Math.sin(robo.sisei[1]+robo.d_omega) + co.obser.y*Math.cos(robo.sisei[1]+robo.d_omega) + robo.location.x;
			co.obserW.y = -co.obser.x*Math.cos(robo.sisei[1]+robo.d_omega) + co.obser.y*Math.sin(robo.sisei[1]+robo.d_omega) + robo.location.y;
		}
		else if(robo.sumA != 0 && robo.d_omega != 0){ //旋回
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
