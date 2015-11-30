import java.io.*;
import java.util.Random;
import java.awt.geom.*;

public class MakeGridmap { //単位系はm
	double x,y;
	int i,xcnt,ycnt;
	int index_x,index_y;
	final double resolution = 0.01; //グリッド間隔
	final int width = 101; //x方向のグリッド数
	final int height = 101; //y方向のグリッド数
	final int gridMax = width*height;
	final double centerX = 0.5; //中心のx座標
	final double centerY = 0; //中心のy座標
	final double startX = 0;
	final double startY = -0.5;
	Grid[] gm = new Grid[gridMax]; //グリッドマップ
	
	MakeGridmap(){ //コンストラクタ
		for(ycnt=0;ycnt<height;ycnt++){
			y = ycnt*resolution+startY;
			for(xcnt=0;xcnt<width;xcnt++){
				x = xcnt*resolution+startX;
				i = height*ycnt + xcnt;
				gm[i] = new Grid(x,y); //グリッドマップオブジェクトの初期化
			}
		}
	}
	
	public void UpdateGrid(Point2D.Double world,int flag){ //グリッドの状態を更新するメソッド
		index_x = (int)((world.x - centerX)/resolution - width/2);
		index_y = (int)((world.y - centerY)/resolution - height/2);
		i = (height*index_y + index_x) + (gridMax - 1);
		gm[i].UpdateGrid(flag);
	}
	
	public void SetFloorStatus(Point2D.Double robo,Point2D.Double obst){
		int i;
		Point2D.Double b,c,grid_i;
		double otoa;
		double slope,slope2,slope3; //傾き
		double ic,ic2,ic3; //切片
		double f,f2,f3; //領域を作っている関数
		b = new Point2D.Double(); c = new Point2D.Double();
		grid_i = new Point2D.Double();
		if(robo.x == obst.x){
			index_x = (int)((obst.x - centerX)/resolution - width/2);
			index_y = (int)((obst.y - centerY)/resolution - height/2);
			i = (height*index_y + index_x) + (gridMax - 1);
			
			b.setLocation(robo.x,robo.y+1);
			c.setLocation(robo.x,robo.y-1);
			slope = (b.y-robo.y)/(b.x-robo.x);
			slope2 = (c.y-robo.y)/(c.x-robo.x);
			ic = robo.y - slope*robo.x;
			ic2 = robo.y - slope2*robo.x;
			
			for(i=0;i<gridMax;i++){
				grid_i = gm[i].ReturnZahyou();
				f = slope*grid_i.x + ic; f2 = slope2*grid_i.x + ic2; f3 = obst.x;
				if(f2<grid_i.y && grid_i.y<f && grid_i.x<f3){
					gm[i].UpdateGrid(1); //領域に囲まれているところを床確定にする
				}
			}
		}
		else if(robo.y == obst.y){
			b.setLocation(robo.x-1,robo.y);
			c.setLocation(robo.x+1,robo.y);
			slope = (b.y-robo.y)/(b.x-robo.x);
			slope2 = (c.y-robo.y)/(c.x-robo.x);
			ic = robo.y - slope*robo.x;
			ic2 = robo.y - slope2*robo.x;
			
			for(i=0;i<gridMax;i++){
				grid_i = gm[i].ReturnZahyou();
				f = slope*grid_i.x + ic; f2 = slope2*grid_i.x + ic2; f3 = obst.y;
				if(f<grid_i.y && f2<grid_i.y && grid_i.y<f3){
					gm[i].UpdateGrid(1); //領域に囲まれているところを床確定にする
				}
			}
		}
		else{
			otoa = (robo.y - obst.y)/(robo.x - obst.y);
			if(otoa < 0){
				b.setLocation(robo.x-1,robo.y+1);
				c.setLocation(robo.x+1,robo.y-1);
				slope = (b.y-robo.y)/(b.x-robo.x);
				slope2 = (c.y-robo.y)/(c.x-robo.x);
				slope3 = (c.y-b.y)/(c.x-b.x);
				ic = robo.y - slope*robo.x;
				ic2 = robo.y - slope2*robo.x;
				ic3 = b.y - slope3*b.x;
				
				for(i=0;i<gridMax;i++){
					grid_i = gm[i].ReturnZahyou();
					f = slope*grid_i.x + ic; f2 = slope2*grid_i.x + ic2; f3 = slope3*grid_i.x + ic3;
					if(f2<grid_i.y && grid_i.y<f && grid_i.y<f3){
						gm[i].UpdateGrid(1); //領域に囲まれているところを床確定にする
					}
				}
			}
			else{
				b.setLocation(robo.x-1,robo.y-1);
				c.setLocation(robo.x+1,robo.y+1);
				slope = (b.y-robo.y)/(b.x-robo.x);
				slope2 = (c.y-robo.y)/(c.x-robo.x);
				slope3 = (c.y-b.y)/(c.x-b.x);
				ic = robo.y - slope*robo.x;
				ic2 = robo.y - slope2*robo.x;
				ic3 = b.y - slope3*b.x;
				
				for(i=0;i<gridMax;i++){
					grid_i = gm[i].ReturnZahyou();
					f = slope*grid_i.x + ic; f2 = slope2*grid_i.x + ic2; f3 = slope3*grid_i.x + ic3;
					if(f<grid_i.y && grid_i.y>f2 && grid_i.y<f3){
						gm[i].UpdateGrid(1); //領域に囲まれているところを床確定にする
					}
				}
			}
		}
	}
	
	public void GaussianUpdate(Point2D.Double obser){ //ガウス関数による確率の重みづけ
		int i,obserGrid;
		double sigma=0.1;
		double distance; //観測点と各グリッドとの距離
		double odds;
		
		for(i=0;i<gridMax;i++){
			distance = obser.distance(gm[i].ReturnZahyou());
			if(distance <= 0.05){
				odds = Math.exp(-Math.pow(distance, 2)/(2*Math.pow(sigma, 2)))/(Math.sqrt(2*Math.PI)*sigma);
				gm[i].UpdateGrid(odds);
			}
		}
	}

	
	public void PrintGrid(){ //グリッドマップの情報（x,y,障害物の確率）をファイル出力するメソッド
		try{
			PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter("Grid.dat")));
			for(i=0;i<gridMax;i++){
				gm[i].PrintGrid(pw);
			}
			pw.close();
		} catch(IOException e){
			System.out.println(e);
		}		
	}
}
