#include <math.h>
#include <stdio.h>
#include <iostream>     // std::cout
#include <algorithm>    // std::remove
#include <stdlib.h> 
#include <iterator> 
#include <cmath>
#include <list>
#include <vector>

using namespace std;

	double BestLength = 0;
	int Iteration;
	double Sump;
	int nextmove = 0;
	double Length;
	 

	vector< vector<double> > h = null;
	vector< vector<double> > CustomerDistance = _distances;
	vector< vector<double> > t = null;
	vector< vector<double> > Customers = _destinations;
	
	int SizeCustomers = sizeof(Customers)/sizeof(Customers(0));

	 try {
                h = new double[SizeCustomers, SizeCustomers];
                t = new double[SizeCustomers, SizeCustomers];
            } catch (std::bad_alloc){
	}
	


	int demand[] = _demands;
	int subdemand;
	subdemand= Sum();
	double extratrips;
	double Nan;

	extratrips=Nan;
	double Capacity =250;
	extratrips = Ceiling(sumdemand / Capacity);
        int vehiclesrequired = std::stoi (str_dec,&extratrips);


	double NearNb = 0;
        double t0 = 0;
	
	int BestTour[] = new int[SizeCustomers + vehiclesrequired];
	
	vector< vector<double> > save = new double[SizeCustomers, SizeCustomers];
























//364-636 den xreiazetai


//371-433


for (int i=1 ; i< sizeof(touriteration)/sizeof(touriteration[0]);i++){

         if (touriteration[i] == 0) {
                        custofveh.insert(custuntilnow);
                        capofveh.insert(capuntilnow);
                        capuntilnow = 0;
                        custuntilnow = 0;
                    } else {
                        custuntilnow += 1;
                        capuntilnow = capuntilnow + demand[touriteration[i]];
                    }
    

}

std::list<int> vehicletours = new std::list<int>();
int veh = 0;
vehicletours.insert(new std::list<int>());
vehicletours[0].insert(0);


		       
for (int i = 1;  i< sizeof(touriteration)/sizeof(touriteration[0]); i++) {
    if (touriteration[i] == 0) {
        vehicletours[veh].insert(0);
        veh += 1;
        vehicletours.insert(new std::list<int>());
        vehicletours[veh].insert(0);
    } else {
        vehicletours[veh].insert(touriteration[i]);
    }
}

  	vehicletours[vehiclesrequired - 1].insert(0);
	std::list<int> LoadofVeh = new std::list<int>();

for (int i = 0; i < vehiclesrequired; i++) {
                    int loadofvehicle = 0;
                    for (int j = 0; j < std::count(vehicletours.begin(),vehicletours.end(),i; j++) {
                        loadofvehicle = loadofvehicle + demand[vehicletours[i][j]];
                    }
                    LoadofVeh.insert(loadofvehicle);
}


double Templength(pow(BestLength,10);
bool foundswap = true;
int tries = 0;

int tour1;
int tour2;
do {
	tour1 = rand()%((vehiclesrequired-1) - 0 + 1) + 0 ;   // rand()%(max-min + 1) + min;
	tour2 = rand()%((vehiclesrequired-1) - 0 + 1) + 0 ;

   }  while (tour1==tour2);

	
`	std::list<int>  fromtour = new std::list<int>();
        std::list<int>  totour = new std::list<int>();




//158-363


for (int i = 0; i < SizeCustomers; i++) {

                for (int j = 0; j < SizeCustomers; j++) {


                    t0 = (1 / ((NearNb * SizeCustomers)));
                    t[i, j] = t0;

                }
            }

	Iteration = 1;
	double tmax = 0;
	tmax = (1 / ((1 - r))) * (1 / NearNb);
        double tmin = 0;
	 tmin = tmax * (1 - pow(0.05, 1 / SizeCustomers)) / ((SizeCustomers / 2 - 1) * pow(0.05, 1 / SizeCustomers));
	double TotalRandomLength[] = new double[500];
	for(int g = 0; g<500; g++){
		load=0;
		double RandomLength = 0;
		std::list<int> RandomUnvisited = new std::list<int>();
		int Start = 0;
		int Randomtour[] = new int[SizeCustomers + vehiclesrequired];
`		Randomtour[0] = Start;
		for (int l = 0; l < SizeCustomers; l++) {
                    RandomUnvisited.insert(l);

                }
		RandomUnvisited = std ::remove(RandomUnvisited,Start)
		bool randomlistempty = false;
                int countrandom = 1;
		while (randomlistempty == false) {
                    int foundnode = 0;
                    for (int i = 0; i < std::count(RandomUnvisited); i++) {
                         int Next = rand() % std::count(RandomUnvisited - 1) + 0 ;
		if (load + demand[RandomUnvisited[Next]] <= Capacity) {
			foundnode = 1;
                        Randomtour[countrandom] = RandomUnvisited[Next];
                        load = load + demand[RandomUnvisited[Next]];
			RandomUnvisited = std ::remove(RandomUnvisited[Next]);
			RandomLength = RandomLength + CustomersDistance[Randomtour[countrandom - 1], Randomtour[countrandom]];
			 countrandom += 1;
			
			 break;
			}
		}
		 if (foundnode == 0) {
                        load = 0;
                        Randomtour[countrandom] = 0;
                        RandomLength = RandomLength + CustomersDistance[Randomtour[countrandom - 1], Randomtour[countrandom]];
                        countrandom += 1;

                    }
		if(std::count(RandomUnvisited) == 0)
			randomlistempty = true;	

		    }

		TotalRandomLength[g] = RandomLength;
		}
		
	    int meancount = 0;
            double DC = 0;
            for (int g = 0; g < 499; g++) {
                DC = DC + std ::abs (TotalRandomLength[g] - TotalRandomLength[g + 1]);
                meancount += 1;
            }

	    DC = DC / meancount;	
	    double SDC = 0;
            for (int g = 0; g < 499; g++) {
                SDC = SDC + pow((TotalRandomLength[g] - TotalRandomLength[g + 1]) - DC, 2);
            }


             SDC = std::sqrt((SDC / meancount));

	     double Temperature = 0;
	     Temperature = (DC + 3 * SDC) / (log(1 / 0.1));
             int activesolution[] = new int[SizeCustomers + vehiclesrequired];
	     activesolution = BestTour;
             double activeLength = NearNb;

	     while (Iteration < NumItsMax) {
                
                int touriteration[] = new int[SizeCustomers + vehiclesrequired];
                double LengthIteration = pow(NearNb, 10);
		for (int k = 1; k < m + 1; k++) {

                    load = 0;
                    int moves = 0;
                    int tour[] = new int[SizeCustomers + vehiclesrequired];
                    tour[moves] = 0;
                    moves += 1;
                    tour[moves] = rand() % (SizeCustomers - 1) +1 ;
                    load = load + demand[tour[moves]];
                    std::list<int> Unvisited = new std::list<int>();
                    for (int l = 0; l < SizeCustomers; l++)
                        Unvisited.insert(l);
		    Unvisited = std ::remove(Unvisited[tour[0]]);
		    Unvisited = std ::remove(Unvisited[tour[1]]);
		    
		    for (int trip = 1; trip < SizeCustomers + vehiclesrequired - 1; trip++) {
                        int c = tour[trip];
			std::list<int> 	PossibleCustomers = new std::list<int>();
				
		    for (int i = 0; i < std::count(Unvisited); i++) {
                            if (load + demand[Unvisited.at(i)] <= Capacity)
                                PossibleCustomers.insert(Unvisited.at(i));
                        }
			
			if (std::count(PossibleCustomers) == 0) {
                            nextmove = 0;
			} else if (c ==0) {
				nextmove = PossibleCustomers[rand() % (PossibleCustomer - 1) + 0 ];
			} else {
				
				std::list<double> choice = new std::list<double>();
				for (int i = 0; i < std::count(PossibleCustomers); i++)	{
				     
					int j = PossibleCustomers.at(i);
					choice.insert(pow(t[c, j], a) * pow(h[c, j], b) * pow(save[c, j], lamda));
				}
				double random1= ((double) rand() / (RAND_MAX)) + 1;  //random double between 0 and 1
				if (random1 >= 1)
                                	cout <<"error1" ;
                                if (random1 < q0) {

		                        double maxValue = std :: max(choise);
		                        int maxIndex = IndexOf(maxValue,choise);
					 nextmove = PossibleCustomers.at(maxIndex);
				}else{
					 std::list<int> p = new std::list<int>();
					 p.clear();
					 Sump = 0;
					 for (int i = 0; i < std::count(PossibleCustomers); i++) {
						int j = PossibleCustomers.at(i);
						 Sump = Sump + (pow(t[c, j], a) * pow(h[c, j], b) * pow(save[c, j], lamda));
						 p.insert((pow(t[c, j], a) * pow(h[c, j], b) * pow(save[c, j], lamda)));

                                }
				double cumsum = 0;
                                double randomnum = ((double) rand() / (RAND_MAX)) + 1;

				if (randomnum >= 1)
                                    cout << "error1";
				for (int i = 0; i < std :: count(p); i++) 					{
                                    p[i] = p[i] / Sump;
                                    p[i] = cumsum + p[i];
                                    cumsum = p[i];
                                }
				for (int j = 0; j <  std :: count(p) - 1; j++) {
                                    if (randomnum >= p[j] && randomnum < p[j + 1]) {
                                        nextmove = PossibleCustomers.at(j);
                                        break;
                                    }
                                }
				if (nextmove == c) {
                                    nextmove = PossibleCustomers[0];
                                }
                            }
                        }
			 if (nextmove == 0) {
                            load = 0;
                            tour[trip + 1] = nextmove;

                        } else {

                            load = load + demand[nextmove];

                            tour[trip + 1] = nextmove;
                            Unvisited = std ::remove(Unvisited[tour[trip + 1]);
                        }

			t[c, tour[trip + 1]] = std :: max (t[c, tour[trip + 1]] * (1 - x) + x * t0, tmin);
                    }

		    Length = 0;
                    for (int i = 0; i < tour[Length] - 1; i++)
                        Length = Length + CustomersDistance[tour[i], tour[i + 1]];

                    if (Length < LengthIteration) {
                        touriteration = tour;
                        LengthIteration = Length;

                    }

                }	




