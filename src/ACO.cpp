
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>
#include <sstream>
#include "string.h"

using namespace std;

class ACOAlgorithm {
public:
  ACOAlgorithm()
  :gen(rd())
  {
  }

  ACOAlgorithm(std::vector< std::vector<double> > distances)
  :gen(rd())
  {
    _distances = distances;
  }

private:
  std::vector<std::vector<double> > _distances;
  std::vector<std::vector<double> > _destinations;

  int _width;
  int _heigth;
  bool _stopped = false;

  std::vector<int> _optimal;
  double _optimalLength;
  bool _Capacitated = false;

  std::vector<int> _demand;
  std::vector<std::vector<int> > _bestList;

  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen; //Standard mersenne_twister_engine seeded with rd()
  // std::uniform_int_distribution<> dis(1, 6);

  //------------------------------------------------------------------------
  // Private Functions
  //------------------------------------------------------------------------

  std::vector< std::vector<double> > ReadCoords(string DistancesFilename){
    string line;
    ifstream myfile(DistancesFilename);

    int line_no = 0;
    int dim;

    std::vector<int> customers;
    std::vector<std::vector<double> > coords;

    if (myfile.is_open()){
      while ( getline (myfile,line) ){
        line_no++;
        //diavazoume tin grammi 5 apo to txt arxeio pou exei ta dedomena  https://stackoverflow.com/questions/26288145/how-to-read-a-specific-line-from-file-using-fstream-c
        if (line_no == 5) {
          // sLine contains the fifth line in the file.
          // cout << line << '\n';
          stringstream ss;
          string temp;
          ss << line;
          while ( !ss.eof() ) {
            ss >> temp;
            if (std::isdigit(temp[0]) ) {
              dim = std::stoi(temp);
              // cout << dim;
              customers.resize(dim);
              coords.resize(dim);
              for (size_t i = 0; i < coords.size(); i++) {
                coords[i].resize(2);
              }
            }
          }
        }


        if ( (line_no >= 7) && (line_no < 7 + dim) ) {
          // std::cout << line << '\n';
          int a;
          myfile >> a;
          // std::cout << a << std::endl;
          myfile >> coords[a-1][0] >> coords[a-1][1];
          // std::cout << coords[a-1][0] << "  " << coords[a-1][1] << std::endl;
        }
      }
      myfile.close();
    }
    else {
      cout << "Unable to open file";
    }

    return coords;
  }

  std::vector<std::vector<double> > calcDistances(const std::vector<std::vector<double> > coords){
    std::vector<std::vector<double> > result(coords.size());
    for (size_t i = 0; i < result.size(); i++) {
      result[i].resize(result.size());
    }

    for (size_t i = 0; i < coords.size(); i++) {
      for (size_t j = i+1; j < coords.size(); j++) {
        result[i][j] = std::sqrt( std::pow( coords[i][0] - coords[j][0], 2 ) + std::pow(coords[i][1] - coords[j][1], 2 ) );
        result[j][i] = result[i][j];
      }
    }

    // for (size_t i = 0; i < result.size(); i++) {
    //   for (size_t j = 0; j < result.size(); j++) {
    //     if (i==j) {
    //       result[i][j] = 0;
    //     }
    //     result[j][i] = result[i][j];
    //     cout<<result[i][j] << " ";
    //   }
    //   cout << endl;
    // }
    return result;
  }

public:
  std::vector<int> getBestPath(){
      return _optimal;
  }

  double getBestLength(){
      return _optimalLength;
  }

  void RunACS(string DistancesFilename){
    double BestLength = 0;
    int Iteration;
    double Sump;
    int nextmove = 0;
    double Length;

    std::vector< std::vector<double> > h;
    std::vector< std::vector<double> > t;

    // std::vector< std::vector<double> > CustomersDistance = ReadCoords(DistancesFilename);
    // std::vector< std::vector<double> > Customers = _destinations;
    std::vector< std::vector<double> > Customers = ReadCoords(DistancesFilename);
    // std::vector< std::vector<double> > CustomersDistance = calcDistances(Customers);
    std::vector< std::vector<double> > CustomersDistance = _distances; //ReadDistances(DistancesFilename);


    if ( CustomersDistance.empty() ){
      // Stop();
      std::cout << "Exiting" << '\n';
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
    double NumIts = 1000;
    double m = 10;
    double q0 = 0.9;
    double b = 2;
    double r = 0.1;
    double x = 0.1;
    double a = 1;

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
    double min = 100000000000000;
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
      if (_stopped) {
        return;
      }
      std::vector<int> touriteration(SizeCustomers + 1);
      double LengthIteration = std::pow(NearNb,10);

      for (size_t k = 1; k < m + 1 ; k++) {
        int moves = 0;
        std::vector<int> tour(SizeCustomers + 1);
        std::uniform_int_distribution<int> dist(0, SizeCustomers - 1);
        tour[moves] = dist(gen);

        std::vector<int> Unvisited;
        for (size_t l = 0; l < SizeCustomers; l++) {
          Unvisited.push_back(l);
        }
        Unvisited.erase(std::remove(Unvisited.begin(),Unvisited.end(),tour[0]),Unvisited.end());

        for (size_t trip = 0; trip < SizeCustomers - 1; trip++) {
          int c = tour[trip];

          std::vector<double> choice;

          for (size_t i = 0; i < Unvisited.size(); i++) {
            int j = Unvisited[i];
            choice.push_back( std::pow(t[c][j], a) * std::pow(h[c][j], b) );
          }

          std::uniform_real_distribution<double> unif(0,1);
          double random1 = unif(gen);

          if (random1 >= 1) {
            cout<<"ERROR1";
          }
          if (random1 < q0) {
            int maxIndex = std::max_element(choice.begin(),choice.end()) - choice.begin();
            double maxValue = *std::max_element(choice.begin(), choice.end());
            nextmove = Unvisited[maxIndex];
          } else {
            std::vector<double> p;
            p.clear();
            Sump = 0;
            for (size_t i = 0; i < Unvisited.size(); i++) {
              int j = Unvisited[i];
              Sump = Sump + (std::pow( t[c][j], a) * std::pow(h[c][j], b));
              p.push_back(std::pow( t[c][j], a) * std::pow(h[c][j], b));
            }

            double cumsum = 0;
            double randomnum = unif(gen);
            if ( randomnum >= 1) {
              cout<< "ERROR1";
            }
            for (size_t i = 0; i < p.size(); i++) {
              p[i] = p[i] / Sump;
              p[i] = cumsum + p[i];
              cumsum = p[i];
            }
            for (int j = 0; j < p.size() - 1; j++){
              if (randomnum >= p[j] && randomnum < p[j + 1]) {
                nextmove = Unvisited[j];
                break;
              }
            }
          }

          if (nextmove == c) {
            nextmove = Unvisited[0];
          }

          tour[trip+1] = nextmove;
          Unvisited.erase(std::remove(Unvisited.begin(),Unvisited.end(),tour[trip+1]),Unvisited.end());

          t[c][tour[trip+1]] = std::max( t[c][tour[trip + 1]] * (1 - x) + x * t0, tmin);

        }

        tour[tour.size() - 1] = tour[0];
        Length = 0;
        for (size_t i = 0; i < tour.size() - 1; i++) {
          Length = Length + CustomersDistance[tour[i]][tour[i+1]];
        }

        if (Length < LengthIteration) {
          touriteration = tour;
          LengthIteration = Length;
        }
      }

      LengthIteration = 0;
      for (size_t i = 0; i < touriteration.size() - 1; i++) {
        LengthIteration = LengthIteration + CustomersDistance[touriteration[i]][touriteration[i+1]];
      }
      if (LengthIteration < BestLength) {
        BestLength = LengthIteration;
        BestTour = touriteration;
      }

      if (activeLength > LengthIteration) {
        activesolution = touriteration;
        activeLength = LengthIteration;
      } else {
        double C = (LengthIteration - activeLength);
        std::uniform_real_distribution<double> unif(0,1);
        if ( unif(gen) < std::exp(-C / Temperature)) {
          activesolution = touriteration;
          activeLength = LengthIteration;
        }
      }

      Temperature = Temperature * 0.999;

      for (size_t i = 0; i < t.size(); i++) {
        for (size_t j = 0; j < t[0].size(); j++) {
          t[i][j] =  std::max(t[i][j] * (1 - t[i][j] / (tmin + tmax)), tmin);
        }
      }

      tmax =  (1 / ((1 - r))) * (1 / BestLength);
      tmin = tmax * (1 - std::pow(0.05, 1 / SizeCustomers)) / ((SizeCustomers / 2 - 1) * std::pow(0.05, 1 / SizeCustomers));

      double minimum = std::pow( Customers[BestTour[0]][2], 10);
      for (int i = 1; i < BestTour.size(); i++) {
        if (minimum > Customers[BestTour[i]][2]) {
          minimum = Customers[BestTour[i]][2];
        }
      }

      for (int i = 0; i < activesolution.size() - 1; i++)
      t[activesolution[i]][activesolution[i + 1]] = std::min(t[activesolution[i]][activesolution[i + 1]] + (t[activesolution[i]][activesolution[i + 1]] / (tmax + tmin)) * (1 / activeLength), tmax);

      results[Iteration] = BestLength;
      Iteration = Iteration + 1;
    }

    _optimal = BestTour;
    _optimalLength = BestLength;
    std::cout << "Best Path: ";
    for (size_t i = 0; i < BestTour.size(); i++) {
      std::cout << BestTour[i] + 1 << " ";
    }
    std::cout << "Best Length: " << BestLength << '\n';
  }

};

// int main(int argc, char const *argv[]) {
//   ACOAlgorithm aco;
//   aco.RunACS("test.txt");
//   return 0;
// }
