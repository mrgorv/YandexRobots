#include <iostream>
#include <vector>
#include <string>
#include <array>
#include <deque>
#include <algorithm>
using namespace std;

struct Coord {
	int x = 0;
	int y = 0;

	bool operator==(Coord other) {
		return this->x == other.x && this->y == other.y;
	}
};

enum PointStatus {
	EMPTY = -1,
	OBSTACLE = -2
};

enum RobotStatus {
	IDLE,
	PICK,
	DROP
};

using MapGrid = vector<vector<int>>;

vector<Coord> WaveSearch(Coord pos, Coord dest, MapGrid& grid) {
	int movex[4] = {1, 0, -1, 0};
	int movey[4] = {0, 1, 0, -1};
	MapGrid field = grid;
	int step = 0, x, y, nd;
	bool stop = false;
	if (pos.x < 0 || pos.y < 0 || pos.x >= field.size() || pos.y >= field.size()) {
		return {};
	}
	field[pos.y][pos.x] = 0;
	do {
		stop = true;
		for (y = 0; y < field.size(); ++y)
			for (x = 0; x < field.size(); ++x)
				if (field[y][x] == step) {
					for (nd = 0; nd < 4; ++nd) {
						int ny = y + movey[nd];
						int nx = x + movex[nd];
						if (ny >= 0 && ny < field.size() && nx >= 0 && nx < field.size() && field[ny][nx] == PointStatus::EMPTY) {
							field[ny][nx] = step + 1;
							stop = false;
						}
					}
				}
		++step;
	} while (!stop && field[dest.y][dest.x] == EMPTY);
	if (field[dest.y][dest.x] == PointStatus::EMPTY) return {};

	int len = field[dest.y][dest.x];
	x = dest.x;
	y = dest.y;
	vector<Coord> path(len + 1);
	if (dest == pos) return {};
	while (step > 0)
	  {
		path[step].x = x;
	    path[step].y = y;
	    --step;
	    for (nd = 0; nd < 4; ++nd)
	    {
	       int ny = y + movey[nd];
	       int nx = x + movex[nd];
	       if (ny >= 0 && ny < field.size() && nx >= 0 && nx < field.size() && field[ny][nx] == step) {
	          x = nx;
	          y = ny;
	          break;
	     }
	   }
	 }
	 path[0].x = pos.x;
	 path[0].y = pos.y;
	 return path;
}

class Robot {
public:
	Coord& GetPos() {
		return position;
	}
	Coord& GetDest() {
		return destination;
	}
	bool IsVacant() {
		return status == IDLE;
	}
	int GetAssumedTime() {
		int result = 0;
		for (auto p : plan) {
			result += p.size() - 1;
		}
		result += path.size() - path_it - 1;
		return result;
	}
	Robot& AssignRoute(vector<Coord> p) {
		if (p.empty()) return *this;
		/*for (auto a : p) {
			cout << a.x << a.y << ' ';
		}*/
		if (status == IDLE) {
			status = PICK;
			path = p;
		}
		else {
			plan.push_back(p);
		}
		destination = p.back();
		return *this;
	}
	char DecideAction() {
		if (status == IDLE) return 'S';
		if (path_it >= path.size() - 1 && path.size() >= 2) {
			if (status == PICK) return 'T';
			else if (status == DROP) return 'P';
		}
		else {
			if (path[path_it].x < path[path_it + 1].x) return 'R';
			else if (path[path_it].x > path[path_it + 1].x) return 'L';
			else if (path[path_it].y > path[path_it + 1].y) return 'U';
			else return 'D';
		}
		return 'S';
	}
	void DoAction(char a) {
		if (a == 'T' || a == 'P') {
			if (!plan.empty()) {
				path = plan.front();
				plan.pop_front();
				if (a == 'T') status = DROP;
				else status = PICK;
				path_it = 0;
			}
			else {
				path.clear();
				path_it = 0;
				status = IDLE;
			}
		}
		else if (a == 'U') {
			path_it++;
			position.y--;
		}
		else if (a == 'D') {
			path_it++;
			position.y++;
		}
		else if (a == 'R') {
			path_it++;
			position.x++;
		}
		else if (a == 'L') {
			path_it++;
			position.x--;
		}
		else status = IDLE;
	}

private:
	Coord position;
	Coord destination;
	uint16_t path_it = 0;
	vector<Coord> path;
	deque<vector<Coord>> plan;
	RobotStatus status = IDLE;
};

class RobotManager {
public:
	RobotManager(int n) : mapsize(n) {
		city.resize(n);
		for (uint16_t i = 0; i < n; ++i) {
			city[i].resize(n);
		}
	}
	MapGrid& GetMap() {
		return city;
	}
	vector<Robot>& GetRobots() {
		return robots;
	}
	void InitializeRobots(uint8_t n) {
		robots.resize(n);
		for (size_t x = 0; x < n; ++x) {
			robots[x].GetPos().x = (x + 1) * city.size() / (n + 1);
			robots[x].GetPos().y = robots[x].GetPos().x;
			robots[x].GetDest() = robots[x].GetPos();
		}
	}
	void PlaceOrder(Coord start, Coord finish) {
		vector<Coord> route = WaveSearch(start, finish, city);
		vector<int> times (robots.size());
		for (Robot r : robots) times.push_back(r.GetAssumedTime() + WaveSearch(r.GetDest(), start, city).size());
		size_t it = min_element(times.begin(), times.end()) - times.begin();
		robots[it].AssignRoute(WaveSearch(robots[it].GetDest(), start, city)).AssignRoute(route);
	}
private:
	MapGrid city;
	vector<Robot> robots;
	uint16_t mapsize = 0;
};

int main() {
	uint16_t robs = 1;
	uint16_t N = 0, maxtips = 0;
	uint32_t cost = 0;
	string mapline;
	cin >> N >> maxtips >> cost;
	RobotManager manager(N);
	for (uint16_t j = 0; j < N; ++j) {
		cin >> mapline;
		for (uint16_t i = 0; i < N; ++i) {
			mapline[i] == '.' ? manager.GetMap()[j][i] = PointStatus::EMPTY : PointStatus::OBSTACLE;
		}
	}
	uint32_t T = 0, D = 0;
	cin >> T >> D;
	cout << robs << "\n";
	manager.InitializeRobots(robs);
	for (uint16_t c = 0; c < robs; ++c) {
		cout << manager.GetRobots()[c].GetPos().x << ' ' << manager.GetRobots()[c].GetPos().y << "\n";
	}
	int k = 0;
	for (uint32_t u = 0; u < T; ++u){
		cin >> k;
		uint16_t srow, scol, frow, fcol;
		for (int l = 0; l < k; ++l) {
			cin >> srow >> scol >> frow >> fcol;
			manager.PlaceOrder({scol-1, srow-1}, {fcol-1, frow-1});
		}
		for (Robot r : manager.GetRobots()) {
			for (uint8_t t = 0; t < 60; ++t) {
				char a = r.DecideAction();
				cout << a;
				r.DoAction(a);
			}
			cout << endl;
		}
	}
	/*uint16_t N = 4;
	RobotManager manager(N);
	for (uint16_t j = 0; j < N; ++j) {
		for (uint16_t i = 0; i < N; ++i) {
			manager.GetMap()[j][i] = PointStatus::EMPTY;
		}
	}
	manager.InitializeRobots(1);
	manager.PlaceOrder({0, 0}, {3, 3});*/	
	return 0;
}
