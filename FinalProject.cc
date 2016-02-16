#include <libplayerc++/playerc++.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <fstream>
#include <algorithm>
#include <vector>
#include <math.h>
#include "lodepng.h"


using namespace PlayerCc;
using namespace std;


//----------- GRID KEY -----------------


// 0	~	FREE SPACE
// 1	~	WALL
// 2	~	TRAVELLED SPACE
// 3	~	SEEN SPACE
// 4	~	SEEN OBSTRUCTION (NOT WALL)
// 6+	~	POSSIBLE STARS


//----------- CONSTANTS -----------------


const int UP = 0;
const int DOWN = 1;
const int RIGHT = 2;
const int LEFT = 3;
const int NOACTION = 4;
const int UNDEFINED = 6;
const int INFINI = 88;
//heading corresponding to facing right
const double RIGHTY = 0.0;
//heading corresponding to facing up
const double UPY = 1.57079;
//heading corresponding to facing left
const double LEFTY = 3.141592;
//heading corresponding to facing down
const double DOWNY = 4.712388;
//threshold value for distance from new value
const double THRESH = 0.0004;
//pixels per meter
const double PXM = 48.4;
//number of star points until we're convinced
const int STARTHRESH = 10;
//accounts for border on PNG(size of border in pixels)
//border should be 8
const int border = 8;

//determines size of grid compared to PNG(pixels/cell)
//larger value means less detail(recommended value is 24)
const int detail = 24;


//________________________________________ FUNCTIONS ________________________________________
//________________________________________ FUNCTIONS ________________________________________
//________________________________________ FUNCTIONS ________________________________________
//________________________________________ FUNCTIONS ________________________________________



//^^^^^^^^^^^^^^^^^^^^^^^^^^^^ FUNCTION LIBRARY ^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void reader(PlayerClient &robot, Position2dProxy &pp,double &x,double &y,double &w);

void extract_policy(vector<vector<int> > &GRID,vector<vector<int> > &grid,vector<vector<int> > &policy,int NROWS,int NCOLS);

void value_iteration(vector<vector<int> > &grid,vector<vector<int> > &navigation,int goalrow,int goalcol,int NROWS,int NCOLS);

void starFinder(PlayerClient &robot, Position2dProxy &pp, LaserProxy &lp, int row, int col, vector< vector<int> > &GRID);

void laser2Grid(PlayerClient &robot, Position2dProxy &pp, LaserProxy &lp, int laserIndex, int &laserRow, int &laserCol, int row, int col);

void Fill_Laser_Data(PlayerClient &robot,Position2dProxy &pp,LaserProxy &lp,vector<vector<int> > GRID);

void MoveRight(PlayerClient &robot,Position2dProxy &pp,double dist);

void MoveLeft(PlayerClient &robot,Position2dProxy &pp,double dist);

void MoveUp(PlayerClient &robot,Position2dProxy &pp,double dist);

void MoveDown(PlayerClient &robot,Position2dProxy &pp,double dist);

void Turn(PlayerClient &robot,Position2dProxy &pp,double neww);

void convert(int row,int col,double& x,double& y,int NROWS,int NCOLS);

void backconvert(int& row,int& col,double x, double y,int NROWS,int NCOLS);

int cost(vector<vector<int> > GRID,int row,int col,int action,int NROWS,int NCOLS);

void transition(vector<vector<int> > GRID,int row,int col,int input,int& newrow,int& newcol,int NROWS,int NCOLS);

void PrintGrid(vector<vector<int> > GRID);

void save_grid(vector<vector<int> > GRID,const char* fname);



//----------------------------------- READER --------------------------------------
void reader(PlayerClient &robot, Position2dProxy &pp,double &x,double &y,double &w) {
	robot.Read();
	x = pp.GetXPos();
	y = pp.GetYPos();
	w = pp.GetYaw();
}


//----------------------------------- POLICY MAKER --------------------------------------
// Computes the policy starting from grid and stores the result in policy.
void extract_policy(vector<vector<int> > &GRID,vector<vector<int> > &nav,vector<vector<int> > &policy,int NROWS,int NCOLS) {
	
	for (int r = 0; r < NROWS; r++)
			{
			for (int c = 0; c < NCOLS; c++) //iterate through all state spaces
				{
				int action;
				int cost = (nav[r][c]); //set it equal to smallest value possible of neighbor

				if (r-1 > 0 && nav[r-1][c] < cost)//check value above, if it is smaller thatn node
					{cost = nav[r-1][c];
					action=UP;}
				if (r+1 < NROWS && nav[r+1][c]< cost)//check value below
					{cost = nav[r+1][c];
					action=DOWN;}
				if (c-1 > 0 && nav[r][c-1]< cost)//check value left
					{cost = nav[r][c-1];
					action=LEFT;}
				if (c+1 < NCOLS && nav[r][c+1] < cost)//check value right
					{cost = nav[r][c+1];
					action=RIGHT;}

				if (nav[r][c] == INFINI)
					action = UNDEFINED;

				if (nav[r][c] == 0)
					action = NOACTION;

				policy[r][c]=action;
				}
			}
}//end function


//----------------------------------- VALUE_ITERATION --------------------------------------
// Fills GRID with the navigation function to given by goalrow and goalcol
void value_iteration(vector<vector<int> > &grid,vector<vector<int> > &navigation,int goalrow,int goalcol,int NROWS,int NCOLS) {

	//initialize an infinite cost to all points except goal
	for(int i=0;i<NROWS;i++){
		for(int j=0;j<NCOLS;j++){
			navigation[i][j] = INFINI;
			}
		}
	navigation[goalrow][goalcol] = 0;
	
	//for each action possible from each state, check if the cost of an adjacent state plus cost to move there is less than cost of current state
	int rowtemp,coltemp;
	bool changed = true;
	while(changed){
		changed = false;
		//for all indices
		for(int i=0;i<NROWS;i++) {
			for(int j=0;j<NCOLS;j++) {
				//if it is not a wall (check for walls in current indices)
				if(grid[i][j] != 1) {
					//run through each possible action (checks each neighbor)
					for(int k=0;k<NOACTION;k++) {
						//debugging code
						//cout << "ACTION CONSIDERED[i][j]: " << i << " " << j << " " << k << " " << "cost: " << cost(navigation,i,j,k,NROWS,NCOLS) <<endl;
						//if it is possible to make that action (checks for walls in neighbors)
						if(cost(navigation,i,j,k,NROWS,NCOLS)<INFINI) {
							//fill the temps with the target indices
							transition(grid,i,j,k,rowtemp,coltemp,NROWS,NCOLS);
							//debugging code
							//cout << "temps: " << rowtemp << " " << coltemp << endl;
							if(rowtemp<INFINI) {
								//if the value of nav in target indices plus cost to move is less than current value
								if(navigation[i][j]+cost(navigation,i,j,k,NROWS,NCOLS) < navigation[rowtemp][coltemp]){
									//then replace the current value
									navigation[rowtemp][coltemp] = navigation[i][j]+cost(navigation,i,j,k,NROWS,NCOLS);
									changed = true;
								}//end if4
							}//end if3
						}//end if2
					}//end for k
				}//end if1
			}//end for j
		}//end for i
	}//end while
}//end function


//----------------------------------- STAR FINDER --------------------------------------
//locates potential star sites and adds to their star total
void starFinder(PlayerClient &robot, Position2dProxy &pp, LaserProxy &lp, int row, int col, vector< vector<int> > &GRID) {
	
	int trigger =-1, starx, stary;
	for(int i = 1; i < 87; i++) {	
		if ( ((lp[i] > lp[i-1]) && (lp[i] > lp[i+1])) && (lp[i] < 1.5) && ((lp[i-1]-lp[i+1])*(lp[i-1]-lp[i+1]) < 0.1))
			trigger = i;
	}

	if (trigger != -1 ) {
		cout << "POTENTIAL STAR FOUND! at " << trigger << endl;
		cout << "I am X: "<< pp.GetXPos() << " I AM Y: " << pp.GetYPos() <<endl; 
		//cout << "STARXpos: " << xpos << " STARYpos: " << ypos << endl;
		laser2Grid(robot, pp, lp, trigger, starx, stary, row, col); 
		cout << "NEW STARX: " << starx << " NEW STARY: " << stary << endl;
		if((GRID[starx][stary] != 1) && (GRID[starx][stary] >= 6))
			GRID[starx][stary]+=1;
		else if ((GRID[starx][stary] != 1) && (GRID[starx][stary] < 6))
			GRID[starx][stary]=6;
	}
	else
		cout <<"nothing yet.... " <<endl;
}


//----------------------------------- LASER2GRID --------------------------------------
//returns indices of any object seen
void laser2Grid(PlayerClient &robot, Position2dProxy &pp, LaserProxy &lp, int laserIndex, int &laserRow, int &laserCol, int row, int col) {
	double theta = lp.GetBearing(laserIndex);
	double xpos = pp.GetXPos() + (lp[laserIndex] * cos(theta));
	double ypos = pp.GetYPos() + (lp[laserIndex] * sin(theta));	
	backconvert(laserRow,laserCol,xpos,ypos,row,col);
}


//----------------------------------- LASER READING --------------------------------------
//the robots eyes
void Fill_Laser_Data(PlayerClient &robot,Position2dProxy &pp,LaserProxy &lp,vector<vector<int> > &GRID,int NROWS,int NCOLS) {
	robot.Read();
	double x = pp.GetXPos();
	double y = pp.GetYPos();
	double w = pp.GetYaw();
	double tempx,tempy;
	int r,c,r1,c1;
	//for each laser
	for(int i=0;i<90;i++) {
		laser2Grid(robot,pp,lp,i,r1,c1,NROWS,NCOLS);
		/*
		//find the point where it ends
		tempx = x + lp[i]*cos(lp.GetBearing(i));
		tempy = y + lp[i]*sin(lp.GetBearing(i));
		//and which indices that translates to
		backconvert(r1,c1,tempx,tempy,NROWS,NCOLS);
		*/
		//then set all the indices along that laser as free space
		for(int j=lp[i];j>0;j = j-0.05) {
			laser2Grid(robot,pp,lp,j,r,c,NROWS,NCOLS);
			/*
			tempx = x + j*cos(lp.GetBearing(j));
			tempy = y + j*sin(lp.GetBearing(j));
			backconvert(r,c,tempx,tempy,NROWS,NCOLS);
			*/
			if(GRID[r][c] != 1 && GRID[r][c] < 6) {
				GRID[r][c] = 3;
			}
		}
		//and sets the end as an obstacle if it isn't already a wall
		if(GRID[r1][c1] != 1 && GRID[r1][c1] < 6 && lp[i] < 3.0) {
			GRID[r1][c1] = 4;
		}
	}
}


//----------------------------------- MOTION FUNCTIONS --------------------------------------
//used to move the robot
void MoveRight(PlayerClient &robot,Position2dProxy &pp,double dist) {
	double x,y,w;
	reader(robot,pp,x,y,w);
	double newx = x + dist;

	while((x-newx)*(x-newx) > THRESH)
		{
		pp.GoTo(newx,y,RIGHTY);
		reader(robot,pp,x,y,w);
		cout<<"RIGHT: "<<"x = "<<x<<"	y = "<<y<<"	w = "<<w<<"\n";
		}
}

void MoveLeft(PlayerClient &robot,Position2dProxy &pp,double dist) {
	double x,y,w;
	reader(robot,pp,x,y,w);
	double newx = x - dist;

	while((x-newx)*(x-newx) > THRESH)
		{
		pp.GoTo(newx,y,LEFTY);
		reader(robot,pp,x,y,w);
		cout<<"LEFT: "<<"x = "<<x<<"	y = "<<y<<"	w = "<<w<<"\n";
		}
}

void MoveUp(PlayerClient &robot,Position2dProxy &pp,double dist) {
	double x,y,w;
	reader(robot,pp,x,y,w);
	double newy = y + dist;

	while((y-newy)*(y-newy) > THRESH)
		{
		pp.GoTo(x,newy,UPY);
		reader(robot,pp,x,y,w);
		cout<<"UP: "<<"x = "<<x<<"	y = "<<y<<"	w = "<<w<<"\n";
		}
}

void MoveDown(PlayerClient &robot,Position2dProxy &pp,double dist) {
	double x,y,w;
	reader(robot,pp,x,y,w);
	double newy = y - dist;

	while((y-newy)*(y-newy) > THRESH)
		{
		pp.GoTo(x,newy,DOWNY);
		reader(robot,pp,x,y,w);
		cout<<"DOWN: "<<"x = "<<x<<"	y = "<<y<<"	w = "<<w<<"\n";
		}


}

void Turn(PlayerClient &robot,Position2dProxy &pp,double neww) {
	double x,y,w;
	reader(robot,pp,x,y,w);

	while((w-neww)*(w-neww) > THRESH)
		{
		pp.GoTo(x,y,neww);
		reader(robot,pp,x,y,w);
		while (w < 0) //because 3/2 pi and -1/2 pi are the same thing.
			w+=2*3.14; //add 2 pi to negative contant to make down be 4.71 or (downy)
		cout<<"TURN: "<<"x = "<<x<<"	y = "<<y<<"	w = "<<w<<"\n";	
		}
}


//----------------------------------- CONVERT --------------------------------------
// converts from row,col to player/stage coordinates.
void convert(int row,int col,double& x,double& y,int NROWS,int NCOLS) {
	x = ((border)+(detail/2)+(col*detail))/PXM;
	y = ((border)+(detail/2)+/*((height-(2*border))%24)*/+(NROWS-row-1)*detail)/PXM;
}


//----------------------------------- BACKCONVERT --------------------------------------
//finds the corresponding row and col given coordinates
void backconvert(int& row,int& col,double x, double y,int NROWS,int NCOLS) {
	
	//initializations
	int cellrow,cellcol;
	double tempx,tempy,tempdist;
	double mindist = INFINI;
	
	//finds node that is the minimum distance from x and y
	for(int temprow=0;temprow<NROWS;temprow++){
		for(int tempcol=0;tempcol<NCOLS;tempcol++){
			convert(temprow,tempcol,tempx,tempy,NROWS,NCOLS);
			tempdist = sqrt((tempx-x)*(tempx-x) + (tempy-y)*(tempy-y));
			if(tempdist<mindist){
				mindist=tempdist;
				cellrow = temprow;
				cellcol = tempcol;
			}
		}
	}
	
	//stores final minimum distance node
	row = cellrow;
	col = cellcol;
}


//----------------------------------- COST --------------------------------------
// returns the cost of applying the given action when on the cell (row,col)
int cost(vector<vector<int> > GRID,int row,int col,int action,int NROWS,int NCOLS) {
	
	int targetrow,targetcol;
	
	switch(action) {
	case UP:
		targetrow = row-1;
		targetcol = col;
		break;
	case DOWN:
		targetrow = row+1;
		targetcol = col;
		break;
	case LEFT :
		targetcol = col-1;
		targetrow = row;
		break;
	case RIGHT:
		targetcol = col+1;
		targetrow = row;
		break;
	}
	if ((targetrow < 0) || (targetcol < 0) || (targetrow >= NROWS) || (targetcol >= NCOLS) || (GRID[targetrow][targetcol] == 1))
		return INFINI;
	else
		return 1;
}


//----------------------------------- TRANSITION --------------------------------------
// Computes the new state (newrow,newcol) obtained applying the given input
void transition(vector<vector<int> > GRID,int row,int col,int input,int& newrow,int& newcol,int NROWS,int NCOLS) {
switch(input){
case UP:
	if(row <= 0){
		newrow = INFINI;
		newcol = INFINI;
		}
	else if(GRID[row-1][col] != 0){
		newrow = INFINI;
		newcol = INFINI;
		}
	else{
		newrow = row-1;
		newcol = col;
		}
	break;
case DOWN:
	if(row >= NROWS-1){
		newrow = INFINI;
		newcol = INFINI;
		}
	else if(GRID[row+1][col] != 0){
		newrow = INFINI;
		newcol = INFINI;
		}
	else{
		newrow = row+1;
		newcol = col;
		}
	break;
case LEFT:
	if(col <= 0){
		newrow = INFINI;
		newcol = INFINI;
		}
	else if(GRID[row][col-1] != 0){
		newrow = INFINI;
		newcol = INFINI;
		}
	else{
		newrow = row;
		newcol = col-1;
		}
	break;
case RIGHT:
	if(col >= NCOLS-1){
		newrow = INFINI;
		newcol = INFINI;
		}
	else if(GRID[row][col+1] != 0){
		newrow = INFINI;
		newcol = INFINI;
		}
	else{
		newrow = row;
		newcol = col+1;
		}
	break;
case NOACTION:
	newrow = row;
	newcol = col;
	break;
	}
}


//----------------------------------- PRINT GRID --------------------------------------
//prints given grid to console
void PrintGrid(vector<vector<int> > GRID) {
	int NROWS = GRID.size();
	int NCOLS = GRID[0].size();
	cout<<endl;
	for(int i=0;i<NROWS;i++) {
		for(int j=0;j<NCOLS;j++) {
			cout<< GRID[i][j] << " ";
		}
		cout<<endl;
	}
	cout<<endl;
}


//----------------------------------- SAVE_GRID --------------------------------------
//saves a given grid to a file
void save_grid(vector<vector<int> > GRID,const char* fname) {
	//find value for rows and cols
	int row = GRID.size();
	int col = GRID[0].size();
	//initialize 2d grid to be saved to a file
	int **grid = new int*[row];
	for ( int i = 0 ; i < row ; i++ ) {
		grid[i] = new int[col];
	}
	//move GRID vector into grid to be saved to file
	for(int i=0;i<row;i++) {
		for(int j=0;j<col;j++) {
		grid[i][j] = GRID[i][j];
		}
	}
	//save to file
	ofstream ofs(fname);
	for(int i=0;i<row;i++) {
		for(int j=0;j<col;j++) {
			ofs << grid[i][j] << " ";
		}
		ofs << endl;
  	}
  	ofs.close();
}





/*
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$	MAIN FUNCTION	$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$	MAIN FUNCTION	$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$	MAIN FUNCTION	$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$	MAIN FUNCTION	$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$	MAIN FUNCTION	$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
*/





int main(int argc, char **argv)
{
	
	
//########################### PNG READING ################################


	//read_png_file(argv[1]); //1 and so forth are the things to be passed on the commandline 
	//e.g. "./pngReader cave.png odometry_x odometry_Y
	// cout << argv[0] << endl;     //this will print out "./pngReader"
	char* filename = argv[1];
	std::vector<unsigned char> image; //will hold the raw pixel data
	unsigned width, height;
	//puts raw data in image and checks for error
	unsigned error = lodepng::decode(image, width, height, filename);
	//if there's an error, display it
	if(error)
		std::cout << "decoder error " << error << ": " << lodepng_error_text(error) << std::endl;
	//the pixels are now in the vector "image", 4 bytes per pixel, ordered RGBARGBA...
	/*
	//		PIXEL PRINTING
	//prints image vector in all of it's glory
	int j =0, k=0, l=0;
	for(int i=0;i<image.size();i+=4) 
	{
		if (((i/4)%484) == 0)
		{
			l++;
			if ( l%24 == 0)
			{
				k++;
			}
			cout << "********" << endl;
		}
		cout << (int)(image[i]) << " ";
	}
	*/



// INITIALIZATIONS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	//row and col should be equal
	const int row = ((height-(2*border))/detail);
	const int col = ((width-(2*border))/detail);
	//initialize GRID as an array or a vector(comment one out)
	//int GRID [row][col];
	vector<vector<int> > GRID(row,std::vector<int>(col,-10));
	vector<vector<int> > NAV(row,std::vector<int>(col,-10));
	vector<vector<int> > POL(row,std::vector<int>(col,-10));
	vector<vector<int> > STAR(row,std::vector<int>(col,0));
	
	
// PRINT PARAMETERS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	/*
	cout << "width: " << width << endl;
	
	cout << "height: " << height << endl;
	
	cout << "pixels: " << image.size() << endl;
	*/
	cout << "row: " << row << endl;
	
	cout << "col: " << col << endl;
	
	
// DECODING PNG~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	//converts 1d image vector into GRID 2d vector with R values of grid pixels
	int q = border+(detail/2);
	for(int i=0;i<row;i++) {
		for(int j=0;j<col;j++) {
			GRID[i][j] = image[4*((q+j*detail)+width*(q+i*detail))];
		}
	}
	
	//converts RED data to 0(free space) and 1(obstacle) and prints GRID
	for(int i=0;i<row;i++) {
		for(int j=0;j<col;j++) {
			if(GRID[i][j] > 0)
				GRID[i][j] = 0;
			else
				GRID[i][j] = 1;
		}
	}

	//we bring the stars out, and the walls out. and the women and the cars out
	STAR = GRID; 

	
// PRINT AND SAVE GRID~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	//prints GRID to console
	PrintGrid(GRID);
	//saves grid to file
	save_grid(GRID,"grid.txt");

	
//**************************PNG HAS BEEN TRANSLATED TO GRID******************************
//***************************************************************************************
//***************************************************************************************


//########################### GRAPH/PLAYER COORDINATE SYNC ################################
		
	
	//processes the initial position of the robot given from the command line
	int r,c;
	double x,y,w;
	double X = atof(argv[2]);
	double Y = atof(argv[3]);
	double W_init = atof(argv[4]);
	double X_bottomLeft = atof(argv[5]);
	double Y_bottomLeft = atof(argv[6]);
	double X_init = X-X_bottomLeft;
	double Y_init = Y-Y_bottomLeft;
	
	//		COORDINATE PRINTING
	
	cout << "INITIAL POSITION: " << endl;
	cout << "x,y coordinates_BL: " << X_init << " , " << Y_init << endl;
	backconvert(r,c,X_init,Y_init,row,col);
	cout << "r,c indices: " << r << " , " << c << endl;
	
	
	//check backconvert
	convert(r,c,x,y,row,col);
	cout << "CHECK: " << x << " , " << y << endl;
	
	
	//prints the x,y coordinates of corners of the grid
	convert(0,0,x,y,row,col);
	cout << "TL: " <<"x(c=" << 0 << "): " << x << " ";
	cout << "y(r=" << 0 << "): " << y << endl;
	convert(row-1,0,x,y,row,col);
	cout << "BL: " << "x(c=" << 0 << "): " << x << " ";
	cout << "y(r=" << row-1 << "): " << y << endl;
	convert(0,col-1,x,y,row,col);
	cout <<"TR: " << "x(c=" << col-1 << "): " << x << " ";
	cout << "y(r=" << 0 << "): " << y << endl;
	convert(row-1,col-1,x,y,row,col);
	cout <<"BR: " << "x(c=" << col-1 << "): " << x << " ";
	cout << "y(r=" << row-1 << "): " << y << endl;
	
	/*
	//prints out every node's corresponding coordinates
	for(int i=0;i<row;i++) {
		for(int j=0;j<col;j++) {
			convert(i,j,x,y,row,col);
			cout << "x(c=" << j << "): " << x << "\t";
			cout << "y(r=" << i << "): " << y << endl;
		}
	}
	*/
	
	
	// setup player
	int Z;
	PlayerClient robot("127.0.0.1", 6665);
	Position2dProxy pp(&robot,0);
	LaserProxy lp(&robot,0);
	while (lp.GetCount() == 0) {
			robot.Read();
			Z = lp.GetCount();
		}
	pp.SetMotorEnable(true);
	robot.Read();
	pp.SetOdometry(X_init,Y_init,W_init);
	reader(robot,pp,x,y,w);
	cout << "Laser Count: " << Z << endl;
	
	
//**********************************ODOMETRY IS SET**************************************
//**********************CAN CONVERT BETWEEN COORIDNATES AND indices**********************
//***************************************************************************************
	
	
//########################### PATH PLANNING ################################
	
	
	//initializations
	reader(robot,pp,x,y,w);
	cout << "x,y,w : " << x << "," << y << "," << w << endl;
	double newx,newy;
	int newrow,newcol,goalr=0,goalc=0;
	bool spacechk = true,starchk = true;
	
	//all motion of the robot happens here
	while(spacechk && starchk) {
		//set checks to false
		spacechk = false;
		starchk = true;


		//update plan
		value_iteration(GRID,NAV,goalr,goalc,row,col);
		save_grid(NAV,"navigation.txt");
		extract_policy(GRID,NAV,POL,row,col);
		save_grid(POL,"policy.txt");
		//update pose
		reader(robot,pp,x,y,w);
		//find current indices
		backconvert(r,c,x,y,row,col);
		GRID[r][c] = 2;
		while(POL[r][c] < NOACTION) {
			if (lp[87] < 0.3)
				MoveDown(robot,pp,(detail/PXM)/2);

			//determines coordinates/indices robot will go to next
			transition(GRID,r,c,POL[r][c],newrow,newcol,row,col);
			cout << "POLICY: " << POL[r][c] << " r: " << r << " c: " << c << " newrow: " << newrow<< " newcol: " << newcol << endl;
			convert(newcol,newrow,newx,newy,row,col);
			//basically just chooses a yaw to go to when it goes to coordinates determined above
			switch(POL[r][c]) {
			case UP:
starFinder(robot, pp, lp, row, col, STAR); //acts on GRID itself
		save_grid(STAR,"Constellations.txt");
				MoveUp(robot,pp,(detail/PXM)/2);
				reader(robot,pp,x,y,w);
				pp.GoTo(newx,newy,UPY);
				backconvert(r,c,x,y,row,col);
				GRID[r][c] = 2;
				break;
			case DOWN:
starFinder(robot, pp, lp, row, col, STAR); //acts on GRID itself
		save_grid(STAR,"Constellations.txt");
				MoveDown(robot,pp,(detail/PXM)/2);
				reader(robot,pp,x,y,w);
				pp.GoTo(newx,newy,DOWNY);
				backconvert(r,c,x,y,row,col);
				GRID[r][c] = 2;
				break;
			case LEFT:	
starFinder(robot, pp, lp, row, col, STAR); //acts on GRID itself
		save_grid(STAR,"Constellations.txt");			
				MoveLeft(robot,pp,(detail/PXM)/2);		
				reader(robot,pp,x,y,w);
				pp.GoTo(newx,newy,LEFTY);
				backconvert(r,c,x,y,row,col);
				GRID[r][c] = 2;
				break;
			case RIGHT:
starFinder(robot, pp, lp, row, col, STAR); //acts on GRID itself
		save_grid(STAR,"Constellations.txt");
				MoveRight(robot,pp,(detail/PXM)/2);
				reader(robot,pp,x,y,w);
				pp.GoTo(newx,newy,RIGHTY);
				backconvert(r,c,x,y,row,col);
				GRID[r][c] = 2;
				break;
			}
			/*
			//update pose
			//reader(robot,pp,x,y,w);
			//look for the star
			Turn(robot,pp,DOWNY);
			starFinder(robot, pp, lp, row, col, STAR);
			Turn(robot,pp,LEFTY);
			starFinder(robot, pp, lp, row, col, STAR);
			Turn(robot,pp,UPY);
			starFinder(robot, pp, lp, row, col, STAR);
			Turn(robot,pp,RIGHTY);
			starFinder(robot, pp, lp, row, col, STAR);
			PrintGrid(STAR);
			*/
		}// end action while
		//checking for free untravelled space
		for(int i=row-1;i >= 0;i--) {
			for(int j=col-1;j>=0;j--) {
			if(GRID[i][j] == 0) {
				spacechk = true;
				goalr = i;
				goalc = j;
				}
			}
		}
		//checking star values over threshold
		for(int i=0;i<row;i++) {
			for(int j=0;j<col;j++) {
			if(STAR[i][j] >= STARTHRESH) starchk = false;
			}
		}
		save_grid(GRID,"grid.txt");


	
	}//end while

	int StarPlace = 0;
	int currenti = 0;
	int currentj = 0;
	int lasers[7] = {0,1,43,44,45,87,88};
	while(StarPlace < 80) { //main looooops

	robot.Read();

	int currX = pp.GetXPos();
	int currY = pp.GetYPos();
	int Yaw = pp.GetYaw();

	cout << "current X: " << currX << endl;
	cout << "current Y: " << currY << endl;
	cout << "current YAW: " << Yaw << endl;
	cout << "Right(0): " << lp[0] << endl;
	cout << "Right(1): " << lp[1] << endl;
	cout << "Straight Ahead[43]: " << lp[43] << endl;
	cout << "Straight Ahead[44]: " << lp[44] << endl;
	cout << "Straight Ahead[45]: " << lp[45] << endl;
	cout << "Left(87): " << lp[87] << endl;	
	cout << "Left(88): " << lp[88] << endl;
	//int laserCol;
	//int laserRow;

/*

	if 	((lp[45] > 1) && (lp[87] < 1))
		MoveForward(robot,pp,1,Yaw);
	else if ((lp[45] > 1) && (lp[87] > 1))
		MoveRLeft(robot,pp,1,Yaw);
	else if ((lp[45] < 1) && (lp[87] > 1))
		MoveRLeft(robot,pp,1,Yaw);
	else if ((lp[45] < 1) && (lp[87] < 1))
		MoveRRight(robot,pp,1,Yaw);
		
*/







	
	/*
	for (int i = 0; i < 7; i++){
		laser2Grid(robot, pp, lp, lasers[i], laserRow, laserCol, row, col);
		cout << "Row" << laserRow << " col" << laserCol << "  For laser: " << lasers[i] <<endl;
		if (GRID[laserRow][laserCol] == 1)
			cout << "Open Space, There is a wall out there" <<endl;
		else
			{
			cout << " unexpected obstacle " <<endl;
			if (GRID[laserRow][laserCol] == 1;			
				GRID[laserRow][laserCol] = 3;
			}
		
		}
	
	*/


	cout << "Right(0): " << lp[0] << endl;
	cout << "Right(1): " << lp[1] << endl;
	cout << "Straight Ahead[40]: " << lp[40] << endl;
	cout << "Straight Ahead[39]: " << lp[39] << endl;
	cout << "Straight Ahead[38]: " << lp[38] << endl;
	cout << "Straight Ahead[41]: " << lp[41] << endl;
	cout << "Straight Ahead[42]: " << lp[42] << endl;
	cout << "Straight Ahead[43]: " << lp[43] << endl;
	cout << "Straight Ahead[44]: " << lp[44] << endl;
	cout << "Straight Ahead[45]: " << lp[45] << endl;
	cout << "Straight Ahead[46]: " << lp[46] << endl;
	cout << "Straight Ahead[47]: " << lp[47] << endl;
	cout << "Straight Ahead[48]: " << lp[48] << endl;
	cout << "Straight Ahead[49]: " << lp[49] << endl;
	cout << "Straight Ahead[50]: " << lp[50] << endl;
	cout << "Straight Ahead[51]: " << lp[51] << endl;
	cout << "Straight Ahead[52]: " << lp[52] << endl;
	cout << "Straight Ahead[53]: " << lp[53] << endl;
	cout << "Straight Ahead[54]: " << lp[54] << endl;
	cout << "Straight Ahead[55]: " << lp[55] << endl;
	cout << "Left(87): " << lp[87] << endl;	
	cout << "Left(88): " << lp[88] << endl;
	cout << "Left(89): " << lp[89] << endl;	
	cout << "Left(90): " << lp[90] << endl;


	starFinder(robot, pp, lp, row, col, STAR); //acts on GRID itself
	save_grid(STAR,"Constellations.txt");
	save_grid(GRID,"GRID.txt");

	for (int i = 1; i < row - 1; i++)
	{
		for (int j = 1; j < col - 1; j++)
		{
			if( (STAR[i-1][j] +  STAR[i+1][j] +  STAR[i][j-1] +  STAR[i][j+1] + STAR[i][j] + STAR[i+1][j+1] + STAR[i+1][j-1] + STAR[i-1][j-1] + STAR[i-1][j+1]) > StarPlace )
			{	
				StarPlace = (STAR[i-1][j] +  STAR[i+1][j] +  STAR[i][j-1] +  STAR[i][j+1] + STAR[i][j] + STAR[i+1][j+1] + STAR[i+1][j-1] + STAR[i-1][j-1] + STAR[i-1][j+1]);
				currenti = i;
				currentj = j;
			}
		}
	}
	cout << "Most possible star position:" << currenti << ", " << currentj << endl;
	cout << StarPlace << endl;
	}//endeverything


	PrintGrid(GRID);
	PrintGrid(NAV);
	PrintGrid(POL);
	PrintGrid(STAR);
	save_grid(GRID,"grid.txt");
	save_grid(NAV,"navigation.txt");
	save_grid(POL,"policy.txt");
	save_grid(STAR,"Constellations.txt");
	
return 0;
}
