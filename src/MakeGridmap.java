import java.io.*;

public class MakeGridmap {
	double x,y;
	int i;
	int index_x,index_y,index;
	final double resolution = 1; //グリッド間隔
	final int width = 101; //x方向のグリッド数
	final int height = 101; //y方向のグリッド数
	final int gridMax = width*height;
	final int centerX = 50; //中心のx座標
	final int centerY = 0; //中心のy座標
	Grid[] gm = new Grid[gridMax]; //グリッドマップ
	
	public MakeGridmap(){ //コンストラクタ
		for(y=-50;y<=50;y++){ //グリッドマップの各要素を初期化
			index_y = (int)((y-centerY)/resolution - height/2);
			for(x=0;x<=100;x++){
				index_x = (int)((x-centerX)/resolution - width/2);
				index = (width*index_x + index_y) + (gridMax - 1);
				gm[index] = new Grid(x,y);
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
