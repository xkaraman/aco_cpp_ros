
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>

using namespace std;

class ACOAlgorithm {
public:
  ACOAlgorithm():gen(rd()){
  }

  ACOAlgorithm(std::vector<std::vector<double> > distances):gen(rd()){
    _distances = distances;
  }

private:
  std::vector<std::vector<double> > _distances;
  std::vector<std::vector<double> > _destinations;

  int _width;
  int _heigth;
  bool _stopped = false;

  std::vector<int> optimal;
  bool _Capacitated = false;

  std::vector<int> _demand;
  std::vector<std::vector<int> > _bestList;

  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen; //Standard mersenne_twister_engine seeded with rd()
  // std::uniform_int_distribution<> dis(1, 6);

  //------------------------------------------------------------------------
  // Private Functions
  //------------------------------------------------------------------------
  std::vector< std::vector<double> > ReadDistances(string DistancesFilename){
      ifstream inFile;
      inFile.open(DistancesFilename.c_str());

      int SizeCustomers = 0;
      string line;

      if (inFile.is_open()) {
        while (getline(inFile,line)) {
          SizeCustomers++;
        }
      }
      inFile.close();

      // double **_Customers;
      std::vector< std::vector<double> > _Customers;
      //_destinations = new double[SizeCustomers,3];
      // _distances = new double*[SizeCustomers];
      _distances.resize(SizeCustomers);
      // int i;
      // for (i = 0; i < SizeCustomers; i++) {
      //   _distances[i] = new double[SizeCustomers];
      // }

      inFile.open(DistancesFilename.c_str());
      for (size_t i = 0; i < _distances.size(); i++) {
        _distances[i].resize(SizeCustomers);
        for (size_t j = 0; j < _distances[i].size(); j++) {
          if (inFile.is_open()) {
            inFile >> _distances[i][j];
            cout << _distances[i][j] << "|";
          }
        }
        cout << endl;
      }

      inFile.close();
      return _Customers;
  }

  public:
    void RunACS(string DistancesFilename){
    double BestLength = 0;
    int Iteration;
    double Sump;
    int nextmove = 0;
    double Length;

    std::vector< std::vector<double> > h;
    std::vector< std::vector<double> > t;

    // double CustomersDistance = _distances; //ReadDistances(DistancesFilename);
    std::vector< std::vector<double> > CustomersDistance = ReadDistances(DistancesFilename);
    std::vector< std::vector<double> > Customers = _destinations;

    if ( CustomersDistance.empty() ){
      // Stop();
      return;
    }

    int SizeCustomers = CustomersDistance.size();
    h.resize(SizeCustomers);
    t.resize(SizeCustomers);
    for (size_t i = 0; i < SizeCustomers; i++) {
      h[i].resize(SizeCustomers);
      t[i].resize(SizeCustomers);
    }

    double NearNb;
    double t0 = 0;
    std::vector<int> BestTour(SizeCustomers + 1);

    for (int i = 0; i < SizeCustomers; i++)
        for (int j = 0; j < SizeCustomers; j++) {
            if (i == j)
                CustomersDistance[i][j] = 1000000000000000000.0;
            h[i][j] = 1 / CustomersDistance[i][j];
        }

    //TODO READ FROM PARAM
    double NumIts;
    double m;
    double q0;
    double b;
    double r;
    double x;
    double a;

    int NumItsMax = NumIts;

    int NextNode = 0;
    std::vector<double> results;
    results.resize(NumItsMax);

    for (size_t i = 0; i < SizeCustomers -1 ; i++) {
      BestLength = BestLength + CustomersDistance[i][i+1];
    }

    // TODO MAX NUM
    // TODO RANDOM NUMBER GENERATOR
    std::uniform_int_distribution<> dis(0, SizeCustomers - 1);
    double min = 10000000000000000;
    int Startingnode = dis(gen);
    std::vector<int> NBUnvisited;
    BestTour[0] = Startingnode;

    for (size_t i = 0; i < SizeCustomers; i++) {
      NBUnvisited.push_back(i);
    }

    // REMOVE ELEMENT WITH VALUE 'Startingnode' FROM VECTOR NBUnvisited
    NBUnvisited.erase(std::remove(NBUnvisited.begin(),NBUnvisited.end(),Startingnode),NBUnvisited.end());

    // COUNT in C# == SIZE in C++
    for (size_t i = 0; i < NBUnvisited.size(); i++) {
        if ( min > CustomersDistance[Startingnode][NBUnvisited[i] ] ) {
          min = CustomersDistance[Startingnode][NBUnvisited[i] ];
          NextNode = NBUnvisited[i];
        }
    }

    NearNb = NearNb + CustomersDistance[Startingnode][NextNode ];
    NBUnvisited.erase(std::remove(NBUnvisited.begin(),NBUnvisited.end(),NextNode),NBUnvisited.end());
    BestTour[1] = NextNode;
    Startingnode = NextNode;

    int count = 1;
    bool listempty = false;

    while ( listempty == false) {
      count++;
      min = 100000000;
      for (size_t i = 0; i < NBUnvisited.size(); i++) {
        if (min > CustomersDistance[Startingnode][NBUnvisited[i]]) {
            min = CustomersDistance[Startingnode][NBUnvisited[i]];
            NextNode = NBUnvisited[i];
        }
      }
      NearNb = NearNb + CustomersDistance[Startingnode][NextNode];
      NBUnvisited.erase(std::remove(NBUnvisited.begin(),NBUnvisited.end(),NextNode),NBUnvisited.end());
      BestTour[count] = NextNode;
      Startingnode = NextNode;
      if (NBUnvisited.size() == 0 ) {
        listempty = true;
      }
    }

    BestTour[count + 1] = BestTour[0];
    NearNb = NearNb + CustomersDistance[BestTour[count]][BestTour[count + 1]];

    for (int i = 0; i < SizeCustomers; i++) {
      for (int j = 0; j < SizeCustomers; j++) {
        t0 = (1 / ((NearNb * SizeCustomers)));
        t[i][j] = t0;
      }
    }

	  Iteration = 1;
	  double tmax = 0;
    tmax = (1 / ((1 - r))) * (1 / NearNb);
    double tmin = 0;
    tmin = tmax * (1 - pow(0.05, 1 / SizeCustomers)) / ((SizeCustomers / 2 - 1) * pow(0.05, 1 / SizeCustomers));

    double TotalRandomLength[500];
    for(int g = 0; g < 500; g++){
      //load=0;
      double RandomLength = 0;
      // std::list<int> RandomUnvisited = new std::list<int>();
      std::vector<int> RandomUnvisited;
      int Start = dis(gen);
      std::vector<int> Randomtour; Randomtour.resize(SizeCustomers + 1);
      Randomtour[0] = Start;
      for (int l = 0; l < SizeCustomers; l++) {
        RandomUnvisited.push_back(l);
      }
      RandomUnvisited.erase(std::remove(RandomUnvisited.begin(),RandomUnvisited.end(),NextNode),RandomUnvisited.end());

      bool randomlistempty = false;
      int countrandom = 1;

      while (randomlistempty == false) {
        std::uniform_int_distribution<int> dist(0, RandomUnvisited.size() - 1);
        int Next = dist(gen);
        Randomtour[countrandom] = RandomUnvisited[Next];
        RandomUnvisited.erase(std::remove(RandomUnvisited.begin(),RandomUnvisited.end(), RandomUnvisited[Next]),RandomUnvisited.end());
        if (RandomUnvisited.size() == 0) {
          randomlistempty= true;
        }

        RandomLength = RandomLength + CustomersDistance[Randomtour[countrandom-1]][Randomtour[countrandom]];
        countrandom +=1;
      }

      Randomtour[countrandom] = Randomtour[0];
      RandomLength = RandomLength + CustomersDistance[Randomtour[countrandom - 1]][Randomtour[countrandom]];

      TotalRandomLength[g] = RandomLength;
    }

    int meancount = 0;
    double DC = 0;
    for (size_t g = 0; g < 499; g++) {
      DC = DC + std::abs(TotalRandomLength[g] - TotalRandomLength[g + 1]);
      meancount += 1;
    }

    DC = DC / meancount;
    double SDC = 0;
    for (int g = 0; g < 499; g++) {
        SDC = SDC + std::pow((TotalRandomLength[g] - TotalRandomLength[g + 1]) - DC, 2);
    }

    SDC = std::sqrt((SDC / meancount));

    double Temperature = 0;
    Temperature = (DC + 3 * SDC) / (std::log(1 / 0.1));
    std::vector<int> activesolution(SizeCustomers + 1);
    activesolution = BestTour;
    double activeLength = NearNb;

    // ACO ITERATIONS
    while (Iteration < NumItsMax) {
      /* code */
    }

  }

};

int main(int argc, char const *argv[]) {
  ACOAlgorithm aco;
  aco.RunACS("test.txt");
  return 0;
}
