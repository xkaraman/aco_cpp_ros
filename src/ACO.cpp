
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
using namespace std;

class ACOAlgorithm {
public:
  ACOAlgorithm(){
  }

  ACOAlgorithm(std::vector<std::vector<double> > distances){
    _distances = distances;
  }

private:
  std::vector<std::vector<double> > _distances;
  std::vector<std::vector<double> > _destinations;

  int _width;
  bool _stopped = false;

  std::vector<int> optimal;
  bool _Capacitated = false;

  std::vector<int> _demand;
  std::vector<std::vector<int> > _bestList;

  //------------------------------------------------------------------------
  // Private Functions
  //------------------------------------------------------------------------
  std::vector<std::vector<double> > ReadDistances(string DistancesFilename){
      ifstream inFile;
      inFile.open(DistancesFilename.c_str());

      int SizeCustomers = 0 ;
      string line;
      if (inFile.is_open()) {
        while (getline(inFile,line)) {
          SizeCustomers++;
        }
      }
      inFile.close();

      // double **_Customers;
      std::vector<std::vector<double> > _Customers;
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
    double bestLength = 0;
    int iteration;
    double sump;
    int nextMove = 0;
    double lentgh;

    std::vector<std::vector<double> > h;
    std::vector<std::vector<double> > t;

    // double **CustomersDistance = ReadDistances(DistancesFilename);
    std::vector<std::vector<double> > CustomersDistance = ReadDistances(DistancesFilename);
    std::vector<std::vector<double> > Customers = _destinations;

    if (CustomersDistance.empty()){
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
    std::vector<int> BestTour(SizeCustomers+1);

    for (int i = 0; i < SizeCustomers; i++)
        for (int j = 0; j < SizeCustomers; j++) {
            if (i == j)
                CustomersDistance[i, j] = 1000000000000000000;
            h[i, j] = 1 / CustomersDistance[i, j];
        }

    int NumItsMax = NumIts;

  }

};

int main(int argc, char const *argv[]) {
  ACOAlgorithm aco = ACOAlgorithm();
  aco.RunACS("test.txt");
  return 0;
}
