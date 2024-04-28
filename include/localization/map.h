#pragma once

#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <sstream>

#include "helper_functions.h"

// Convert from world coords to map coords
#define MAP_GXWX(map, x) (floor((x - map->origin_x) / map->scale + 0.5))
#define MAP_GYWY(map, y) (floor((y - map->origin_y) / map->scale + 0.5))

// Test to see if the given map coords lie within the absolute map bounds.
#define MAP_VALID(map, col, row) ((col >= 0) && (col < map->size_x) && (row >= 0) && (row < map->size_y))

// Compute the cell index for the given map coords.
#define MAP_INDEX(map, col, row) ((col) + (row) * map->size_x)

struct Map
{
	float origin_x, origin_y;

	float scale;

	int size_x, size_y;

	std::vector<int> map_points;

	Map() {origin_x = 0.0; origin_y = 0.0; scale = 0.0; size_x = 0; size_y = 0; map_points.clear(); }
};

inline Map map_load_occ(const std::string &filename, float scale, float origin_x, float origin_y) {    
    std::ifstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + filename);
    }

	Map map;
	map.scale = scale;
	map.origin_x = origin_x;
	map.origin_y = origin_y;
	
	std::string line;
	int row_count = 0;
    int col_count = 0;

	while (std::getline(file, line)) {
        std::stringstream ss(line);
        int value = 0;
		col_count = 0;

        while (ss >> value) {
            map.map_points.push_back(value);
            col_count++;
        }

		row_count++;
	}

	map.size_x = col_count; // col
	map.size_y = row_count; // line

	file.close();

	return map;
}

inline float map_calc_range(Map* map, float ox, float oy, float oa, float max_range, float resolution){
    // Bresenham raytracing
    int x0,x1;
    int y0,y1;
    int x,y;
    int xstep, ystep;
    int steep;
    int tmp;
    int deltax, deltay, error, deltaerr;

	// Restore resolution [cm] -> [pixel]
	ox = ox / resolution;
	oy = oy / resolution;
	max_range = max_range / resolution;

	// Particle location
	x0 = static_cast<int>(ox);
	y0 = static_cast<int>(oy);
    oa = normalize_0_to_2pi(oa);

	// Finish point
	x1 = static_cast<int>(ox + max_range * cos(oa));
	y1 = static_cast<int>(oy + max_range * sin(oa));
	
	if(abs(y1-y0) > abs(x1-x0))
		steep = 1;
	else
		steep = 0;

	if(steep)
	{
		tmp = x0;
		x0 = y0;
		y0 = tmp;

		tmp = x1;
		x1 = y1;
		y1 = tmp;
	}

	deltax = abs(x1-x0);
	deltay = abs(y1-y0);
	error = 0;
	deltaerr = deltay;

	x = x0;
	y = y0;

	if(x0 < x1)
		xstep = 1;
	else
		xstep = -1;
	if(y0 < y1)
		ystep = 1;
	else
		ystep = -1;

	if(steep)
	{
		if(!MAP_VALID(map,y,x) || map->map_points[MAP_INDEX(map,y,x)] > 0) {
			return dist(x0, y0, x, y) * resolution;
		}
	}
	else
	{
		if(!MAP_VALID(map,x,y) || map->map_points[MAP_INDEX(map,x,y)] > 0) {
			return dist(x0, y0, x, y) * resolution;
		}
	}

	while(x != (x1 + xstep * 1))
	{
		x += xstep;
		error += deltaerr;
		if(2*error >= deltax)
		{
		y += ystep;
		error -= deltax;
		}

		if(steep)
		{
		if(!MAP_VALID(map,y,x) || map->map_points[MAP_INDEX(map,y,x)] > 0){
			return dist(x0, y0, x, y) * resolution;
		}
		}
		else
		{
		if(!MAP_VALID(map,x,y) || map->map_points[MAP_INDEX(map,x,y)] > 0){
			return dist(x0, y0, x, y) * resolution;
		}
		}
	}


    return max_range * resolution;
}

struct Crosswalk {
	float x; // Left Down x Position
	float y; // Left Down y Position
	int w; // Cross Walk Weight
	int h; // Cross Walk Height

	Crosswalk() {x = 0.0; y = 0.0; w = 0; h = 0;}
};

inline std::vector<Crosswalk> cross_walk_load_occ(const std::string &filename) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + filename);
    }

	std::vector<Crosswalk> cross_walk_areas;
	std::string line;

	while (std::getline(file, line)) {
		Crosswalk cw;
		std::stringstream ss(line);

		ss >> cw.x >> cw.y >> cw.w >> cw.h;

		cross_walk_areas.push_back(cw); 
	}

	file.close();

	return cross_walk_areas;
}