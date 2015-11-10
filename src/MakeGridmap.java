import java.io.*;

public class MakeGridmap {
	double x,y;
	int i,xcnt,ycnt;
	int index_x,index_y,index;
	final double resolution = 0.01; //グリッド間隔
	final int width = 101; //x方向のグリッド数
	final int height = 101; //y方向のグリッド数
	final int gridMax = width*height;
	final double centerX = 0.5; //中心のx座標
	final double centerY = 0; //中心のy座標
	final double startX = 0;
	final double startY = -0.5;
	Grid[] gm = new Grid[gridMax]; //グリッドマップ
	
	public MakeGridmap(){ //コンストラクタ
		for(i=0;i<gridMax;i++){
			gm[i] = new Grid();
		}
		for(ycnt=0;ycnt<height;ycnt++){
			y = ycnt*resolution+startY;
			index_y = (int)((y-centerY)/resolution - height/2);
			for(xcnt=0;xcnt<width;xcnt++){
				x = xcnt*resolution+startX;
				index_x = (int)((x-centerX)/resolution - width/2);
				index = (width*index_x + index_y) + (gridMax - 1);
				gm[index].SetCoodinate(x,y);
			}
		}
	}
	
	public void UpdateGrid(double worldX,double worldY,int flag){ //グリッドの状態を更新するメソッド
		index_x = (int)((worldX - centerX)/resolution - width/2);
		index_y = (int)((worldY - centerY)/resolution - height/2);
		index = (index_x*width+index_y) + (gridMax - 1);
		gm[index].UpdateGrid(flag);
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
