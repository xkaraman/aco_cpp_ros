#ifndef ACO_H
#define ACO_H

#include <string>
#include <vector>
#include <random>


class ACOAlgorithm{
public:
  ACOAlgorithm();
  ACOAlgorithm(std::vector< std::vector < double> > distances );

  std::vector<int> getBestPath();
  double getBestLength();

  void setParameters(double maxIterations = 3000,
					  double numAnts = 20,
					  double explorationProbability = 0.9,
					  double exhaustRatePheromones = 0.1,
					  double transitionProbabilityB = 2,
					  double transitionProbabilityX = 0.1,
					  double transitionProbabilityA = 1);

  void RunACS(std::string DistancesFilename);

private:
  std::vector<std::vector<double> > _distances;
  std::vector<std::vector<double> > _destinations;

  // ACO Parameters
  double _maxIterations = 3000;
  double _numAnts = 20;
  double _explorationProbability = 0.9;
  double _exhaustRatePheromones = 0.1;
  double _transitionProbabilityB = 2;
  double _transitionProbabilityX = 0.1;
  double _transitionProbabilityA = 1;

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

  std::vector< std::vector<double> > readCoords(std::string DistancesFilename);
  std::vector< std::vector<double> > calcDistances(const std::vector<std::vector<double> > coords);

};

#endif
