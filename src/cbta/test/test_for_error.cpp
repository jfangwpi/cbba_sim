/*
 *  json_access.hpp
 *
 *  area to determine source of error when 
 * 	running CBTA on the Raspberry Pi
 * 
 *  Created on: June 4th, 2019
 *      Author: Jack O'Neill
 *      Questions/comments: (203) 558-2221
 */

// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>
#include <bitset>
// opencv
#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>

// user
#include "vis/graph_vis.hpp"
#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "map/square_grid.hpp"
#include "vis/graph_vis.hpp"

#include "ltl/buchi_automaton.hpp"
#include "ltl/product_automaton.hpp"
#include "cbta/graph_lift.hpp"
#include "cbta/hcost_interface.hpp"
#include "nlohmann/json.hpp"

using namespace Eigen;
using namespace librav;

int main(int argc, char** argv){
	std::string connectivity_str;

	const int curr_H = 3;
	const int rmin = 3;
	const int tile_num = 0;

	std::string folder = "./H"+std::to_string(curr_H)+"_R"+std::to_string(rmin)+"/";
	std::ifstream i_temp(folder+"H"+std::to_string(curr_H)+"_R"+std::to_string(rmin)+"_Tile"+std::to_string(tile_num)+".txt");
			nlohmann::json j_tile;
			i_temp >> j_tile;

    std::stringstream str;
    str << "Tile_" << tile_num;
    std::string title = str.str();
    const char *current_tile_name = title.c_str();

	j_tile.at(current_tile_name).at("connectivity").get_to(connectivity_str);


	// *** create and manipulate connectivity matrix ***//
	char conn_char[connectivity_str.size() +1];
	strcpy(conn_char, connectivity_str.c_str());
	char *token;
	std::vector<char *> burner_array;
	token = strtok (conn_char,";");
	int counter = 0;
	while (token != NULL)
	{	
		burner_array.push_back(token);
		token = strtok (NULL, ";");
		counter++;
	}

	MatrixXi edge_list = MatrixXi::Zero(counter,3);

	for (size_t i = 0; i < counter; i++)
	{	
		std::vector<int> ind_num_array;
		char *token;
		token = strtok (burner_array[i],",");
		std::stringstream ss1;
		ss1 << token;
		int idx1;
		ss1 >> idx1;

		token = strtok (NULL, ",");
		std::stringstream ss2;
		ss2 << token;
		int idx2;
		ss2 >> idx2;
		edge_list(i,0) = idx1;
		edge_list(i,1) = idx2;
		edge_list(i,2) = 1;
	}

	typedef Eigen::Triplet<int> T;
	std::vector<T> tripletList;
	tripletList.reserve(edge_list.rows());
	for (int k = 0; k < edge_list.rows(); k++){
		tripletList.push_back(T(edge_list(k,0),edge_list(k,1),edge_list(k,2))); // T(i,j,v_ij)
	}

	std::shared_ptr<Eigen::SparseMatrix<int,Eigen::RowMajor>> connectivity = std::make_shared<Eigen::SparseMatrix<int,Eigen::RowMajor>>(2500,2500);
	connectivity->setFromTriplets(tripletList.begin(), tripletList.end());
	std::cout << "this was succesfull!" << std::endl;
}
