import java.io.*;
import java.awt.geom.*;

public class Grid {
	private Point2D.Double zahyou = new Point2D.Double(); //グリッド位置
	private int status; //グリッド状態(0:未走査, 1:空間確定, 2:障害物確定, 3:推定中)
	private double odds; //オッズ値（初期値は1）
	private double exp; //障害物かどうかの確率(0:空間〜1:障害物) 初期値は0.5
	
	Grid(double x,double y){
		zahyou.x = x; zahyou.y = y;
		status = 0;
		odds = 1;
		exp = odds/(1+odds);
	}
	
	public Point2D.Double ReturnZahyou(){
		return zahyou;
	}
	
	public int ReturnStatus(){
		return status;
	}
	
	public double ReturnExp(){
		return exp;
	}
	
	public void UpdateGrid(double newOdds){
		status = 3;
		odds += newOdds;
		exp = odds/(1+odds);
	}
	
	public void UpdateGrid(int flag){
		switch(flag){
		case 1:
			status = 1;
			exp = 0;
			break;
		case 2:
			status = 2;
			exp = 1;
			break;
		case 3:
			if(exp == 1){
				break;
			}
			else{
				status = 3;
				exp += 0.1;
				if(exp == 1){
					status = 2;
				}
				break;
			}
		}
	}
	
	public void PrintGrid(PrintWriter pw){
		pw.println(zahyou.x + "\t" + zahyou.y + "\t" + exp);
	}
}
