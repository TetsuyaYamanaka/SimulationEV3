import java.awt.geom.Point2D;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

class SensorMearsure extends TimerTask{
	Point2D.Double obst_vir = new Point2D.Double(0.4,0.2); //障害物座標（ワールド系）を設定
	Point2D.Double obser = new Point2D.Double(),obser2 = new Point2D.Double(); //ロボットから見た観測点
	Point2D.Double obserW = new Point2D.Double(),obserW2 = new Point2D.Double(); //ワールド座標系での観測点
	double[] len = new double[2];
	int timeMax = 5;
	long time0,time;
	int i,flag=0,initFlag=0;
	
	RobotMove rm; //ロボットインスタンス（参照）
	
	SensorMearsure(long startTime){
		time0 = startTime;
	}
	
	public void run(){
		time = TimeUnit.MILLISECONDS.convert(System.nanoTime()-time0, TimeUnit.NANOSECONDS);
		if(time >= (long)timeMax*1000){
			flag=0;
			this.cancel();
		}
		if(initFlag == 0){
			len[1] = obst_vir.distance(rm.robo.location);
			initFlag = 1; //センサ起動フラグを立てる
		}
		else{
			len[0] = len[1];
			len[1] = obst_vir.distance(rm.robo.location);
			flag = 1;
		}
		//System.out.println("sensor: " + time);
	}
	
	public void SetInstance(RobotMove robot){ //ロボットインスタンスへの参照をセット
		rm = robot;
	}
}
