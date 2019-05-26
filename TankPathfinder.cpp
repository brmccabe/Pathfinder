#include "TankPathfinder.h"
#include <iostream>
#include <algorithm> 
#include <iomanip>
#include "playing.h"
#include <fstream>

const int TankPathfinder::neighbor_pos[][2] = { { -1, 0 },{ -1, 1 },{ 0, 1 },{ 1, 1 },{ 1, 0 },{ 1, -1 },{ 0, -1 },{ -1,-1 } };

TankPathfinder::TankPathfinder() : scale(1)
{
	offset = 0;
        pathfound = false;
}


TankPathfinder::~TankPathfinder()
{
	while (!grid.empty()) {
		std::vector<Cell*>* next = grid.at(0);
		while (!next->empty()) {
			Cell* erasing = (next->at(0));
			next->erase(next->begin());
			delete erasing;
		}
		grid.erase(grid.begin());
	}
}

//edit so it deallocates any previous grid
void TankPathfinder::build_grid(int rows, int columns, int sca) {
	dimensions[0] = rows;
	dimensions[1] = columns;
	scale = sca;
	scaled_dimensions[0] = rows / scale;
	scaled_dimensions[1] = columns / scale;
	current_search = 0;
	total_cells = 0;
	adjacent_dist = 10;
	diagonal_dist = 14;
	if (scaled_dimensions[0] > 0 && scaled_dimensions[1] > 0) {
		for (int i = 0; i <= scaled_dimensions[0]; i++) {
			std::vector<Cell*>* next_row = new std::vector<Cell*>;
			for (int j = 0; j <= scaled_dimensions[1]; j++) {
				Cell* next_col = new Cell;
				(*next_col).f_cost = 0;
				(*next_col).g_cost = 0;
				(*next_col).heuristic = 0;
				(*next_col).weight = 0;
				//(*next_col).is_target = false;
				(*next_col).obstacle = false;
				(*next_col).parent = NULL;
				(*next_col).position[0] = i;
				(*next_col).position[1] = j;
				(*next_col).scaled_position[0] = (i * scale) + offset;
				(*next_col).scaled_position[1] = (j * scale) + offset;
				(*next_col).last_visit = 0;
				(*next_col).last_discover = 0;
				(*next_col).in_path = false;
				//(*next_col).discovered = false;
				(*next_row).push_back(next_col);
			}
			grid.push_back(next_row);
		}
		total_cells = grid.size() * grid.at(0)->size();
	}
}

void TankPathfinder::set_target(int x, int y) {
	if ((x - offset) < dimensions[0] && (y - offset) < dimensions[1] && (x - offset) >= 0 && (y - offset) >= 0) {
		Cell *edit_cell = grid.at((x - offset) / scale)->at((y - offset) / scale);
		target = edit_cell;
		edit_cell->obstacle = false;
	}
	else {
		target = NULL;
	}
}

void TankPathfinder::set_offset(int set) {
	offset = set;
}
void TankPathfinder::set_location(int x, int y) {
  if ((x - offset) < dimensions[0] && (y - offset) < dimensions[1] && (x - offset) >= 0 && (y - offset) >= 0) {
		Cell *edit_cell = grid.at((x - offset) / scale)->at((y - offset) / scale);
		location = edit_cell;
		edit_cell->obstacle = false;
	}
	else {
		location = NULL;
	}
}

void TankPathfinder::set_obstacle(int x, int y) {
  if ((x - offset) < dimensions[0] && (y - offset) < dimensions[1] && (x - offset) >= 0 && (y - offset) >= 0) {
		Cell *edit_cell = grid.at((x - offset) / scale)->at((y - offset) / scale);
		if (edit_cell != target && edit_cell != location) {
			edit_cell->obstacle = true;
		}
	}
}

void TankPathfinder::set_weight(int x, int y, int set) {
  if ((x - offset) < dimensions[0] && (y - offset) < dimensions[1] && (x - offset) >= 0 && (y - offset) >= 0) {
		Cell *edit_cell = grid.at((x - offset) / scale)->at((y - offset) / scale);
		if (edit_cell != target && edit_cell != location) {
			edit_cell->weight = set;
		}
	}
}

void TankPathfinder::find_path() {
  pathfound = false;
  if (target != NULL && location != NULL) {
    target->parent = NULL;
    current_search++;
    Cell *current_cell = location;
    Cell *neighbor_cells[8];
    std::vector<Cell*> discovered;
    discovered.push_back(current_cell);
    std::string e = std::to_string(location->position[0]) + ", " + std::to_string(location->position[1]) + "  |  " + std::to_string(target->position[0]) + ", " + std::to_string(target->position[1]);
    controlPanel->addMessage(e);
    int i = 1;
    for (; i < total_cells && current_cell != target && !discovered.empty(); i++) {
      //std::cout << "\n\n\n Now updating neibors for (" << current_cell->position[0] << ", " << current_cell->position[1] << ")\n";
      current_cell->last_visit = current_search;
      get_valid_neighbors(neighbor_cells, current_cell);
      for (int j = 0; j < 8; j++) {
        if (neighbor_cells[j] != NULL) {
          update_cell(current_cell, neighbor_cells[j]);
          if (neighbor_cells[j]->last_discover != current_search) {
            discovered.push_back(neighbor_cells[j]);
            neighbor_cells[j]->last_discover = current_search;
          }
        }
      }
      //current_cell->last_visit = current_search;
      auto find_current = std::find(discovered.begin(), discovered.end(), current_cell);
      if (find_current != discovered.end()) {
        discovered.erase(find_current);
      }
      auto smallest = std::min_element(discovered.begin(), discovered.end(), [](Cell* a, Cell* b) {return a->f_cost < b->f_cost; });
      if (smallest != discovered.end())
        current_cell = *smallest;
    }
    location->parent = NULL;
    pathfound = true;
  }
  else
    controlPanel->addMessage(":(:(:(");
}

void TankPathfinder::get_valid_neighbors(Cell **neighbor_cells, Cell* current_cell) {
	//check if neighbor is on the grid
	for (int i = 0; i < 8; i++) {
		int next_posx = current_cell->position[0] + neighbor_pos[i][0];
                int next_posy = current_cell->position[1] + neighbor_pos[i][1];
		//std::cout << "(" << next_posx << ", " << next_posy << ")\n";
                //if (next_posx > -1) {
                if (next_posx > -1 && next_posx < (int)(grid.size()) && next_posy > -1 && next_posy < (int)(grid.at(0)->size())) {
                        neighbor_cells[i] = grid.at(next_posx)->at(next_posy);
			if (neighbor_cells[i]->obstacle) {         
				neighbor_cells[i] = NULL;
			}
		}
		else {
                  
			neighbor_cells[i] = NULL;
		}
	}
	//check if corner neighbor is blocked by obstacles on both sides
	for (int i = 1; i < 8; i += 2) {
		if (neighbor_cells[i] == NULL || (neighbor_cells[(i + 1) % 8] == NULL && neighbor_cells[(i - 1)%8] == NULL)) {
			neighbor_cells[i] = NULL;
		}
	}
	//check if cell has already been visited
	for (int i = 0; i < 8; i++) {
		if (neighbor_cells[i] != NULL && neighbor_cells[i]->last_visit == current_search) {
			neighbor_cells[i] = NULL;
		}
	}
	/*std::cout << "\n\n";
	for (int i = 0; i < 8; i++) {
		std::cout << "neighbor[" << i << "] is ";
		if (neighbor_cells[i]) {
			std::cout << "(" << neighbor_cells[i]->position[0] << ", " << neighbor_cells[i]->position[1] << ")\n";
		}
		else
			std::cout << "NULL\n";
	}*/
}

void TankPathfinder::update_cell(Cell* current, Cell* update_target) {
	calc_gcost(update_target, current);
	calc_h(update_target);
	update_target->f_cost = update_target->g_cost + update_target->heuristic;
	//update_target->last_discover == current_search;
	//std::cout << update_target->position[0] << ", " << update_target->position[1] << "\n";
	//std::cout << update_target->g_cost << ", " << update_target->f_cost << "\n";
}

void TankPathfinder::calc_gcost(Cell* update_target, Cell* current) {
	int updated_g = current->g_cost + update_target->weight;
	if (current->position[0] == update_target->position[0] || current->position[1] == update_target->position[1]) {
		updated_g += adjacent_dist;
	}
	else {
		updated_g += diagonal_dist;
	}
	if (update_target->last_discover == current_search) {
		if (update_target->g_cost > updated_g) {
			update_target->parent = current;
			update_target->g_cost = updated_g;
		}
	}
	else {
		update_target->g_cost = updated_g;
		update_target->parent = current;
	}
	/*Cell* pare = update_target->parent;
	if(pare)
		std::cout <<"parent of ("<< update_target->position[0] << ", " << update_target->position[1] << ") is (" << pare->position[0]<<", "<< pare->position[1] << ") gcost: "<<update_target->g_cost<< "\n";
*/}

void TankPathfinder::calc_h(Cell* cell) {
	if (cell->last_discover != current_search) {
		int x_dist = abs(target->position[0] - cell->position[0]);
		int y_dist = abs(target->position[1] - cell->position[1]);
		cell->heuristic = 0;
		cell->heuristic += diagonal_dist * std::min(x_dist, y_dist);
		cell->heuristic += adjacent_dist * abs(x_dist - y_dist);
	}
}

void TankPathfinder::print_grid() {
	for (unsigned int j = 0; j < grid.at(0)->size(); j++) {
		for (unsigned int i = 0; i < grid.size(); i++) {
			if (grid.at(i)->at(j)->obstacle) {
				std::cout << std::setw(4) << "X";
			}
			else if (grid.at(i)->at(j) == target) {
				std::cout << std::setw(4) << "T";
			}
			else if (grid.at(i)->at(j) == location) {
				std::cout << std::setw(4) << "S";
			}
			else if (grid.at(i)->at(j)->in_path){
				std::cout << std::setw(4) << "-";
			}
			else {
				std::cout << std::setw(4) << grid.at(i)->at(j)->weight;
			}
		}
		std::cout << "\n";
	}
}

void TankPathfinder::file_grid(std::string name) {
    const int PADDING = 4;
    std::ofstream fout;
    fout.open(name);
    for (unsigned int j = 0; j < grid.at(0)->size(); j++) {
      for (unsigned int i = 0; i < grid.size(); i++) {
        if (grid.at(i)->at(j)->obstacle) {
          fout << std::setw(PADDING) << "XXXX";
        }
        else if (grid.at(i)->at(j) == target) {
          fout << std::setw(40) << "{TTTTTTTTTT}";
        }
        else if (grid.at(i)->at(j) == location) {
          fout << std::setw(40) << "{SSSSSSSSSS}";
        }
        else if (grid.at(i)->at(j)->in_path) {
          fout << std::setw(PADDING) << " -- ";
        }
        else {
          fout << std::setw(PADDING) << " ";
        }
      }
      fout << "\n";
    }
    fout << grid.size();
    fout.close();
}

void TankPathfinder::get_path() {
	path.push_back(target->scaled_position);
	Cell* current = target;
	while (current->parent != NULL) {
		current->in_path = true;
		current = current->parent;
		path.push_front(current->scaled_position);
	}
}
