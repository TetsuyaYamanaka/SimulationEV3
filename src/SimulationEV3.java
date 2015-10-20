import java.io.*;

public class SimulationEV3 {
	static class zahyou{
		double x,y;
	}
	
	public static void main(String[] args) {
		final int datamax = 1000;
		final double tread = 12.2;
		final double hankei = 1.5;
		final double period = 0.01;
		
		int i,cnt = 0;
		double len[] = new double[datamax],len_p[] = new double[datamax],a[] = new double[datamax]; //cm
		double d_theta,theta,theta_p;
		double theta2,theta_p2;
		float v = 4; //車体速度(cm/sec)
		float omega = (float)(0*(Math.PI/180)); //車体姿勢角(rad)
		double time = 0;
		int timemax = 5;
		double sin_theta,sin_theta_p;
		double cos_d_theta;
		float[] sisei = new float[datamax];
		zahyou[] p_p = new zahyou[datamax]; zahyou[] p2_p = new zahyou[datamax]; //前時刻の障害物座標(ロボット系)
		zahyou[] p = new zahyou[datamax]; zahyou[] p2 = new zahyou[datamax]; //現時刻の障害物座標(ロボット系)
		zahyou[] robo = new zahyou[datamax]; //ロボット座標(ワールド系)
		zahyou[] obst = new zahyou[datamax]; zahyou[] obst2 = new zahyou[datamax];//障害物座標(ワールド系)
		
		for(i=0;i<datamax;i++){
			p_p[i] = new zahyou(); p2_p[i] = new zahyou();
			p[i] = new zahyou(); p2[i] = new zahyou();
			robo[i] = new zahyou();
			obst[i] = new zahyou(); obst2[i] = new zahyou();
		}
		
		len[cnt] = 50; a[cnt] = 0; sisei[cnt] = (float)(0*(Math.PI/180));
		robo[cnt].x = 0; robo[cnt].y = 0;
		len_p[cnt+1] = len[cnt];
		cnt++;

		while(time < timemax){
			len[cnt] = len_p[cnt] - 4.0*period;
			a[cnt] = v*period;
			sisei[cnt] = (float)(omega*period + sisei[cnt-1]);
			robo[cnt].x = v*Math.cos(sisei[cnt])*period + robo[cnt-1].x;
			robo[cnt].y = v*Math.sin(sisei[cnt])*period + robo[cnt-1].y;
			len_p[cnt+1] = len[cnt];
			time += period;
			cnt++;
		}
		
		for(i=0;i<cnt;i++){
			cos_d_theta = (len[i]*len[i] + len_p[i]*len_p[i] - a[i]*a[i])/(2*len[i]*len_p[i]);
			d_theta = Math.acos(cos_d_theta); //逆三角関数によりΔθを求める
	
			sin_theta = len_p[i]*Math.sin(d_theta)/a[i]; sin_theta_p = len[i]*Math.sin(d_theta)/a[i];
			theta = Math.asin(sin_theta); theta_p = Math.asin(sin_theta_p); //逆三角関数によりθ、θ´を求める
			theta2 = -theta; theta_p2 = -theta_p;
	
			p_p[i].x = len_p[i]*Math.sin(theta_p); p_p[i].y = len_p[i]*Math.cos(theta_p);
			p2_p[i].x = len_p[i]*Math.sin(theta_p2); p2_p[i].y = len_p[i]*Math.cos(theta_p2);
			p[i].x = len[i]*Math.sin(theta); p[i].y = len[i]*Math.cos(theta);
			p2[i].x = len[i]*Math.sin(theta2); p2[i].y = len[i]*Math.cos(theta2);
			
			obst[i].x = p[i].x*Math.sin(sisei[i]) + p[i].y*Math.cos(sisei[i]) + robo[i].x;
			obst[i].y = -p[i].x*Math.cos(sisei[i]) + p[i].y*Math.sin(sisei[i]) + robo[i].y;
			obst2[i].x = p2[i].x*Math.sin(sisei[i]) + p2[i].y*Math.cos(sisei[i]) + robo[i].x;
			obst2[i].y = -p2[i].x*Math.cos(sisei[i]) + p2[i].y*Math.sin(sisei[i]) + robo[i].y;
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
			
			pw.println(p_p[0].x + "\t" + p_p[0].y + "\t" + p2_p[0].x + "\t" + p2_p[0].y);
			for(i=0;i<cnt;i++){
				pw.println(p[i].x + "\t" + p[i].y + "\t" + p2[i].x + "\t" + p2[i].y);
				pw2.println(obst[i].x + "\t" + obst[i].y + "\t" + obst2[i].x + "\t" + obst2[i].y);
				pw3.println(robo[i].x + "\t" + robo[i].y);
			}
			pw.close();
			pw2.close();
			pw3.close();
		}catch(IOException e){
			System.out.println(e);
		}
	}

}
