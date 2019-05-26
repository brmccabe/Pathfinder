#pragma once
#include <vector>
#include <deque>


class TankPathfinder
{
public:
	struct Cell {
		int g_cost;			//distance from start point
		int heuristic;		//distance from end point
		int f_cost;			//g_cost + h_cost
		bool obstacle;
		int weight;
		//bool discovered;
		int position[2];
		int scaled_position[2];
		Cell* parent;
		long last_visit;
		long last_discover;
		bool in_path;

	};

	TankPathfinder();
	~TankPathfinder();

	void build_grid(int rows, int columns, int sca = 1);
	void set_target(int x, int y);
	void set_location(int x, int y);
	void set_obstacle(int x, int y);
	void set_offset(int set);
	void find_path();
	void update_cell(Cell* current, Cell* update_target);
	void calc_gcost(Cell* cell, Cell* current);
	void calc_h(Cell* cell);
	void get_path();
	void set_weight(int x, int y, int set);
	void print_grid();
	void get_valid_neighbors(Cell **neighbor_cells, Cell* current_cell);
        void file_grid(std::string name);
	//bool valid_neighbor(int* neighbor_pos);
	//bool check_diag(Cell* a, Cell* b);




	std::vector<std::vector<Cell*>*> grid;
	int dimensions[2];
	int scaled_dimensions[2];
	Cell* target;
	Cell* location;
	int scale;
	int offset;
	std::deque<int*> path;
	int diagonal_dist;
	int adjacent_dist;
	long current_search;
	int total_cells;
	static const int neighbor_pos[][2];
        bool pathfound;
};
