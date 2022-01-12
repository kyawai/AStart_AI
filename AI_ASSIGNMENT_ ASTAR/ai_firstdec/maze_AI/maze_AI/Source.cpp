#include <iostream>
#include <fstream>
#include <string>
#include <Windows.h>
#include <stdlib.h>
#include <time.h>
#include <iomanip>
#include <queue>
#include <ctime>


// A star veriables
//global
static int mapArrayAStar[21][21];
static int closedNodeMap[21][21];
static int openNodeMap[21][21];
static int mapDirection[21][21];
const int direction = 8;
static int dx[direction] = { 1,1,0,-1,-1,-1,0,1 };
static int dy[direction] = { 0,1,1,1,0,-1,-1,-1 };

class node
{
	int xPosition, yPosition;// current position
	int level;
	int priority;


public:

	node(int xp, int yp, int d, int p) { xPosition = xp; yPosition = yp; level = d; priority = p; }
	int getxPos() const { return xPosition; }
	int getyPos() const { return yPosition; }
	int getLevel() const { return level; }
	int getPriority() const { return priority; }

		//A*
	void updatePriority(const int & xDestination, const int & yDestination)
	{
		priority = level + estimate(xDestination, yDestination) * 10;
	}
	//gives priority to going in a straight path rather than going diagonally
	void nextLevel(const int & i)
	{
		level += (direction == 8 ? (i % 2 == 0 ? 10 : 14) : 10);
	}
	//estimation function, used for the remaining distance to the goal
	const int & estimate(const int & xDestination, const int & yDestination) const
	{
			static int xd, yd, d;
			xd = xDestination - xPosition;
			yd = yDestination - yPosition;

			//euclidian distance
			d = static_cast<int>(sqrt(xd*xd + yd * yd));

			return(d);
	}
	

};

//determines the prioirty (in the priority queue)
bool operator<(const node & a, const node & b) { return a.getPriority() > b.getPriority(); }

//A* algorith. retrunes route, (string in direction digits)

std::string pathFind(const int & xStartAStar, const int & yStartAStar, const int & xFinishAStar, const int & yFinishAStar, int firstNum, int secondNum)
{
		static std::priority_queue<node> pq[2];//list of open nodes
		static int pqIndex;
		static node* n0;
		static node* m0;
		static int i, j, x, y, xdx, ydy;
		static char c;
		pqIndex = 0;
		char tempMaze;
		char mazeSize;
		char tempFirstNum = 0, tempSecondNum = 0;
		//reset node map
		// load text file
		std::fstream inputMazeAStar("Maze1.txt");

		//if the file is open:

		if (inputMazeAStar.is_open())
		{
				for (int num1 = 1; num1 < 2; num1++)
				{
						inputMazeAStar >> mazeSize;
						if (mazeSize != '\0' && mazeSize != (-1) && mazeSize != (-2) && mazeSize != 32 && mazeSize != (-52))
						{
								// check if there is a second number (double digit size of maze)

								for (int num1two = 1; num1two < 3; num1two++)
								{
										inputMazeAStar >> mazeSize;
										if (mazeSize != '\0' && mazeSize != (-1) && mazeSize != (-2) && mazeSize != 32 && mazeSize != (-52))
										{
												int doubleDigitNum = mazeSize;
										}
										else if (num1two == 2)
										{
												tempFirstNum = tempFirstNum - 48; //take away 48 from char to get the size of the maze
										}
								}
						}
						else
						{
								num1 = num1 - 1;
						}
				}
				for (int num1 = 1; num1 < 2; num1++)
				{
						inputMazeAStar >> mazeSize;
						if (mazeSize != '\0' && mazeSize != (-1) && mazeSize != (-2) && mazeSize != 32 && mazeSize != (-52))
						{
								// check if there is a second number (double digit size of maze)

								for (int num1two = 1; num1two < 3; num1two++)
								{
										inputMazeAStar >> mazeSize;
										if (mazeSize != '\0' && mazeSize != (-1) && mazeSize != (-2) && mazeSize != 32 && mazeSize != (-52))
										{
												int doubleDigitNum = mazeSize;
										}
										else if (num1two == 2)
										{
												tempSecondNum = tempSecondNum - 48; //take away 48 from char to get the size of the maze
										}
								}
						}
						else
						{
								num1 = num1 - 1;
						}
				}
				for (y = 0; y < secondNum; y++)
				{
						for (x = 0; x < firstNum; x++)
						{
	
								inputMazeAStar >> tempMaze;
								if (tempMaze == '0' || tempMaze == '1' || tempMaze == '2' || tempMaze == '3')
								{
										closedNodeMap[x][y] = 0;
										openNodeMap[x][y] = 0;
										tempMaze = tempMaze - 48;
										mapArrayAStar[y][x] = tempMaze;
								}
								else
								{	
										x = x - 1;
								}
						}
				}

				//create start node then push it into a list of open nodes

				n0 = new node(xStartAStar, yStartAStar, 0, 0);
				n0->updatePriority(xFinishAStar, yFinishAStar);
				pq[pqIndex].push(*n0);
				openNodeMap[x][y] = n0->getPriority();

				//A* search

				while (!pq[pqIndex].empty())
				{
						//get the current node with the highest priority from the list of open nodes

						n0 = new node(pq[pqIndex].top().getxPos(), pq[pqIndex].top().getyPos(), pq[pqIndex].top().getLevel(), pq[pqIndex].top().getPriority());
						x = n0->getxPos(); y = n0->getyPos();
						//remove node from open list
						pq[pqIndex].pop();
						openNodeMap[x][y] = 0;
						//mark the node on the closed nodes map
						closedNodeMap[x][y] = 1;

						//stop searching when goal is reached

						if (x == xFinishAStar && y == yFinishAStar)
						{
								//generate the path from start to finish by following the directions
								std::string path = "";
								while (!(x == xStartAStar && y == yStartAStar))
								{
										j = mapDirection[x][y];
										c = '0' + (j + direction / 2) % direction;
										path = c + path;
										x += dx[j];
										y += dy[j];
								}

								//delete
								delete n0;
								//empty leftover nodes
								while (!pq[pqIndex].empty()) pq[pqIndex].pop();
								return path;
						}

						//generate moves in all possible directions
						
						for (i = 0; i < direction; i++)
						{
								xdx = x + dx[i]; ydy = y + dy[i];

								if (!(xdx<0 || xdx>firstNum - 1 || ydy<0 || ydy>secondNum - 1 || mapArrayAStar[ydy][xdx] == 1 || closedNodeMap[xdx][ydy] == 1 ))
								{

										//generate child node
										m0 = new node(xdx, ydy, n0->getLevel(), n0->getPriority());
										m0->nextLevel(i);
										m0->updatePriority(xFinishAStar, yFinishAStar);

										//if not in open list, add it 
										if (openNodeMap[xdx][ydy] == 0)
										{
												openNodeMap[xdx][ydy] = m0->getPriority();
												pq[pqIndex].push(*m0);
												//mark its parent node direction
												mapDirection[xdx][ydy] = (i + direction / 2) % direction;
										}
										else if (openNodeMap[xdx][ydy] > m0->getPriority())
										{
												//update the priority info
												openNodeMap[xdx][ydy] = m0->getPriority();
												//update parent direction ifno
												mapDirection[xdx][ydy] = (i + direction / 2) % direction;

												//replace the node by emptuing one pq to the other
												//except the note to be replaced will be ignored, new node will be pushed in

												while (!pq[pqIndex].top().getxPos() == xdx && pq[pqIndex].top().getyPos() == ydy)
												{
														pq[1 - pqIndex].push(pq[pqIndex].top());
														pq[pqIndex].pop();
												}
												//remove the wanted node
												pq[pqIndex].pop();

												//empty the larger size pq to the smaller one
												if (pq[pqIndex].size() > pq[1 - pqIndex].size()) pqIndex = 1 - pqIndex;
												while (!pq[pqIndex].empty())
												{
														pq[1 - pqIndex].push(pq[pqIndex].top());
														pq[pqIndex].pop();
												}
												pqIndex = 1 - pqIndex;
												//add better node
												pq[pqIndex].push(*m0);
										}
										//delete
										else delete m0;
								}
						}
						//delete
						delete n0;
				}
				return "";
		}
		inputMazeAStar.close();
}


int main()
{
	//initialisation of veriables
	char fileSize = 0;
	char currNumChar; 
	int firstNum = 0, firstNum2 = 0, secondNum = 0, secondNum2 = 0;
	int yFinish, yStart, xStart, xFinish;
	int mazeSize;
	int tempFirstNum = 0, tempSecondNum = 0;
	
	srand(time(NULL));

	// load text file
	std::fstream input("Maze1.txt");

	//if the file is open:
	if (input.is_open())
	{
		//checks the firt sets of numbers to find the size of the maze.
		//row
		for (int num1 = 1; num1 < 2; num1++)
		{
			input >> fileSize;
			if (fileSize != '\0' && fileSize != (-1) && fileSize != (-2) && fileSize != 32 && fileSize != (-52))
			{
				// check if there is a second number (double digit size of maze)
				firstNum = fileSize;
				for (int num1two = 1; num1two < 3; num1two++)
				{
					input >> fileSize;
					if (fileSize != '\0' && fileSize != (-1) && fileSize != (-2) && fileSize != 32 && fileSize != (-52))
					{
						firstNum2 = fileSize;
						if (firstNum2 == '0') { firstNum = firstNum - 48; firstNum = firstNum * 10; }
						else if (firstNum2 == '1') { firstNum = firstNum - 48; firstNum = firstNum * 10 + 1; }
						else if (firstNum2 == '2') { firstNum = firstNum - 48; firstNum = firstNum * 10 + 2; }
						else if (firstNum2 == '3') { firstNum = firstNum - 48; firstNum = firstNum * 10 + 3; }
						else if (firstNum2 == '4') { firstNum = firstNum - 48; firstNum = firstNum * 10 + 4; }
						else if (firstNum2 == '5') { firstNum = firstNum - 48; firstNum = firstNum * 10 + 5; }
						else if (firstNum2 == '6') { firstNum = firstNum - 48; firstNum = firstNum * 10 + 6; }
						else if (firstNum2 == '7') { firstNum = firstNum - 48; firstNum = firstNum * 10 + 7; }
						else if (firstNum2 == '8') { firstNum = firstNum - 48; firstNum = firstNum * 10 + 8; }
						else if (firstNum2 == '9') { firstNum = firstNum - 48; firstNum = firstNum * 10 + 9; }
					}
					else if (num1two == 2)
					{
						firstNum = firstNum - 48; //take away 48 from char to get the size of the maze
					}
				}
			}
			else
			{
				num1 = num1 - 1;
			}
		}
		//above repeated for column
		for (int num1 = 1; num1 < 2; num1++)
		{
			input >> fileSize;
			if (fileSize != '\0' && fileSize != (-1) && fileSize != (-2) && fileSize != 32 && fileSize != (-52))
			{
				secondNum = fileSize;
				for (int num1two = 1; num1two < 3; num1two++)
				{
					input >> fileSize;
					if (fileSize != '\0' && fileSize != (-1) && fileSize != (-2) && fileSize != 32 && fileSize != (-52))
					{
						secondNum2 = fileSize;
						if (secondNum2 == '0') { secondNum = secondNum - 48; secondNum = secondNum * 10; }
						else if (secondNum2 == '1') { secondNum = secondNum - 48; secondNum = secondNum * 10 + 1; }
						else if (secondNum2 == '2') { secondNum = secondNum - 48; secondNum = secondNum * 10 + 2; }
						else if (secondNum2 == '3') { secondNum = secondNum - 48; secondNum = secondNum * 10 + 3; }
						else if (secondNum2 == '4') { secondNum = secondNum - 48; secondNum = secondNum * 10 + 4; }
						else if (secondNum2 == '5') { secondNum = secondNum - 48; secondNum = secondNum * 10 + 5; }
						else if (secondNum2 == '6') { secondNum = secondNum - 48; secondNum = secondNum * 10 + 6; }
						else if (secondNum2 == '7') { secondNum = secondNum - 48; secondNum = secondNum * 10 + 7; }
						else if (secondNum2 == '8') { secondNum = secondNum - 48; secondNum = secondNum * 10 + 8; }
						else if (secondNum2 == '9') { secondNum = secondNum - 48; secondNum = secondNum * 10 + 9; }
					}
					else if (num1two == 2)
					{
						secondNum = secondNum - 48;
					}
				}
			}
			else
			{
				num1 = num1 - 1;
			}

		}

		closedNodeMap[firstNum][secondNum]; openNodeMap[firstNum][secondNum]; mapDirection[firstNum][secondNum]; 
		
		//find start and end pos
		//set the map
		for (int i = 0; i < secondNum; i++)
		{
			for (int j = 0; j < firstNum;	j++)
			{
				
					input >> fileSize;
					mapArrayAStar[i][j] = fileSize;

				if (mapArrayAStar[i][j] == (-1) || mapArrayAStar[i][j] == (-2) || mapArrayAStar[i][j] == (32) || mapArrayAStar[i][j]  == (-52) || mapArrayAStar[i][j] == ('\0'))
				{
					j = j - 1;
				}
				else
				{
						if (mapArrayAStar[i][j] == '2')
						{
								xStart = j;
								yStart = i;
						}

						if (mapArrayAStar[i][j] == '3')
						{
								xFinish = j;
								yFinish = i;
						}
						mapArrayAStar[i][j] = mapArrayAStar[i][j] - 48;
				}
			}
		}


		//get the route
		clock_t start = clock();
		std::string route = pathFind(xStart, yStart, xFinish, yFinish, firstNum, secondNum);
		if (route == "") std::cout << "empty route generated" << std::endl;
		clock_t end = clock();
		double time_elapsed = double(end - start);
		std::cout << " time it took for the route (ms) :  " << time_elapsed << std::endl;
		std::cout << "Route: " << std::endl;
		std::cout << route << std::endl << std::endl;

		//follow route on the map and display it

		if (route.length() > 0)
		{
				int j; char c;
				int x = xStart;
				int y = yStart;
				mapArrayAStar[y][x] = 2;
				for (int i = 0; i < route.length(); i++)
				{
						c = route.at(i);
						j = atoi(&c);
						x = x + dx[j];
						y = y + dy[j];
						mapArrayAStar[y][x] = 3;
				}
				mapArrayAStar[y][x] = 4;
				
		
				for (int x = 0; x < secondNum; x++)
				{
						for (int y = 0; y < firstNum; y++ )
						{

								if (mapArrayAStar[x][y] == 0) { std::cout << "-"; }
								else if (mapArrayAStar[x][y] == 1) { std::cout << "|"; } //wall
								else if (mapArrayAStar[x][y] == 2) { std::cout << "2"; } //start
								else if (mapArrayAStar[x][y] == 3) { std::cout << "R"; } // route
								else if (mapArrayAStar[x][y] == 4) { std::cout << "3"; } //finish

						}
						std::cout << " " << std::endl;
				}
			
		}
	}
	system("Pause");

	return 0;
}

