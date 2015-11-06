import java.io.*;
import java.awt.geom.*;

public class SimulationEV3 {
	public static void main(String[] args) {
		final int datamax = 1000;
		final double tread = 12.2;
		final double hankei = 1.5;
		final double period = 0.01;
		final int floorFlag = 1,obstFlag = 2,estFlag = 3; //（1.床確定 2.障害物確定  3.推定中）
		
		int i,cnt = 0;
		double len[] = new double[datamax],len_p[] = new double[datamax],a[] = new double[datamax]; //cm
		double d_theta,theta;
		double theta2;
		float v = 4; //車体速度(cm/sec)
		float omega = (float)(0*(Math.PI/180)); //車体姿勢角(rad)
		double time = 0;
		int timemax = 5;
		double sin_theta;
		double cos_d_theta;
		float[] sisei = new float[datamax];
		Point2D.Double[] p_p = new Point2D.Double[datamax]; Point2D.Double[] p2_p = new Point2D.Double[datamax];//前時刻の推定障害物座標(ロボット系)
		Point2D.Double[] p = new Point2D.Double[datamax]; Point2D.Double[] p2 = new Point2D.Double[datamax];//前時刻の推定障害物座標(ロボット系)
		Point2D.Double[] robo = new Point2D.Double[datamax]; //ロボット座標(ワールド系)
		Point2D.Double[] obst = new Point2D.Double[datamax]; Point2D.Double[] obst2 = new Point2D.Double[datamax];//推定障害物座標(ワールド系)
		float[] d_omega = new float[datamax]; //1サンプリングタイムの車体回転角
		
		Point2D.Double obst_vir = new Point2D.Double(40,20); //障害物座標（ワールド系）を設定
		MakeGridmap gm = new MakeGridmap();
		
		for(i=0;i<datamax;i++){
			p_p[i] = new Point2D.Double(); p2_p[i] = new Point2D.Double();
			p[i] = new Point2D.Double(); p2[i] = new Point2D.Double();
			robo[i] = new Point2D.Double();
			obst[i] = new Point2D.Double(); obst2[i] = new Point2D.Double();
		}
		
		len[cnt] = obst_vir.distance(robo[cnt]);
		a[cnt] = 0;
		sisei[cnt] = (float)(0*(Math.PI/180));
		d_omega[cnt] = 0;
		len_p[cnt+1] = len[cnt];
		cnt++;

		while(time < timemax){
			a[cnt] = v*period;
			sisei[cnt] = (float)((omega/timemax)*period + sisei[cnt-1]);
			robo[cnt].setLocation(a[cnt]*Math.cos(sisei[cnt]) + robo[cnt-1].x,a[cnt]*Math.sin(sisei[cnt]) + robo[cnt-1].y);
			len[cnt] = obst_vir.distance(robo[cnt]);
			d_omega[cnt] = sisei[cnt] - sisei[cnt-1];
			len_p[cnt+1] = len[cnt];
			time += period;
			cnt++;
		}
		
		for(i=0;i<cnt;i++){
			cos_d_theta = (len[i]*len[i] + len_p[i]*len_p[i] - a[i]*a[i])/(2*len[i]*len_p[i]);
			d_theta = Math.acos(cos_d_theta); //逆三角関数によりΔθを求める

			if(d_omega[i] == 0){ //直進
				sin_theta = len_p[i]*Math.sin(d_theta)/a[i];
				theta = Math.asin(sin_theta);//逆三角関数によりθを求める
				theta2 = -theta;
				
				p[i].x = len[i]*Math.sin(theta); p[i].y = len[i]*Math.cos(theta);
				p2[i].x = len[i]*Math.sin(theta2); p2[i].y = len[i]*Math.cos(theta2);
				
				obst[i].x = p[i].x*Math.sin(sisei[i]) + p[i].y*Math.cos(sisei[i]) + robo[i].x;
				obst[i].y = -p[i].x*Math.cos(sisei[i]) + p[i].y*Math.sin(sisei[i]) + robo[i].y;
				obst2[i].x = p2[i].x*Math.sin(sisei[i]) + p2[i].y*Math.cos(sisei[i]) + robo[i].x;
				obst2[i].y = -p2[i].x*Math.cos(sisei[i]) + p2[i].y*Math.sin(sisei[i]) + robo[i].y;
			}
			else if(a[i] == 0){ //その場で回転
				p[i].x = len[i]*Math.cos(Math.PI/2 + d_omega[i]);
				p[i].y = len[i]*Math.sin(Math.PI/2 + d_omega[i]);
				
				obst[i].x = p[i].x*Math.sin(sisei[i]+d_omega[i]) + p[i].y*Math.cos(sisei[i]+d_omega[i]) + robo[i].x;
				obst[i].y = -p[i].x*Math.cos(sisei[i]+d_omega[i]) + p[i].y*Math.sin(sisei[i]+d_omega[i]) + robo[i].y;
			}
			else{ //旋回
				theta = d_theta + Math.abs(d_omega[i]);
				theta2 = -theta;
				if(d_omega[i] < 0){
					p[i].x = len[i]*Math.sin(theta); p[i].y = len[i]*Math.cos(theta);
				}
				else{
					p[i].x = -len[i]*Math.sin(theta); p[i].y = len[i]*Math.cos(theta);	
				}
			}
			
			/*obst[i].x = p[i].x*Math.sin(sisei[i]) + p[i].y*Math.cos(sisei[i]) + robo[i].x;
			obst[i].y = -p[i].x*Math.cos(sisei[i]) + p[i].y*Math.sin(sisei[i]) + robo[i].y;
			obst2[i].x = p2[i].x*Math.sin(sisei[i]) + p2[i].y*Math.cos(sisei[i]) + robo[i].x;
			obst2[i].y = -p2[i].x*Math.cos(sisei[i]) + p2[i].y*Math.sin(sisei[i]) + robo[i].y;*/
		}
		
		try{
			FileWriter fw = new FileWriter("SimulationP.dat");
			BufferedWriter bw = new BufferedWriter(fw);
			PrintWriter pw = new PrintWriter(bw);
			FileWriter fw2 = new FileWriter("Obstacle.dat");
			BufferedWriter bw2 = new BufferedWriter(fw2);
			PrintWriter pw2 = new PrintWriter(bw2);
			FileWriter fw3 = new FileWriter("Robozahyou.dat");
			BufferedWriter bw3 = new BufferedWriter(fw3);
			PrintWriter pw3 = new PrintWriter(bw3);
			FileWriter fw4 = new FileWriter("Obstacle_virtual.dat");
			BufferedWriter bw4 = new BufferedWriter(fw4);
			PrintWriter pw4 = new PrintWriter(bw4);
			FileWriter fw5 = new FileWriter("Simulationlen.dat");
			BufferedWriter bw5 = new BufferedWriter(fw5);
			PrintWriter pw5 = new PrintWriter(bw5);
			
			pw4.println(obst_vir.x + "\t" + obst_vir.y);			
			for(i=0;i<cnt;i++){
				pw.println(p[i].x + "\t" + p[i].y + "\t" + p2[i].x + "\t" + p2[i].y);
				pw2.println(obst[i].x + "\t" + obst[i].y + "\t" + obst2[i].x + "\t" + obst2[i].y);
				pw3.println(robo[i].x + "\t" + robo[i].y);
				pw5.println(len[i]);
				gm.UpdateGrid(robo[i].x,robo[i].y,floorFlag); //ロボットの走行軌道を床確定に更新
				gm.UpdateGrid(obst[i].x,obst[i].y,obstFlag); //推定障害物位置を障害物確定に更新
				gm.UpdateGrid(obst2[i].x,obst2[i].y,obstFlag); //同上
			}
					
			pw.close();
			pw2.close();
			pw3.close();
			pw4.close();
			pw5.close();
		}catch(IOException e){
			System.out.println(e);
		}
		
		gm.PrintGrid(); //グリッドマップの情報をファイル出力（x,y,障害物確率）
	}
}
