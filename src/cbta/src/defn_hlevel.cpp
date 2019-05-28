/*
 * defn_Hlevel.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: bscooper
 */

// Eigen header
#include <eigen3/Eigen/Core>
#include "cbta/hcost_tile_library.hpp"

using namespace Eigen;
using namespace librav;

/* ------- Hlevel constructors and destructors -------- */
Hlevel::Hlevel(unsigned int Hin){
	H = Hin;
	n_tiles = 0;
	unique_tiles = MatrixXi::Zero(H,1);

}

Hlevel::Hlevel(unsigned int Hin, unsigned int n_tiles_in, std::string unique_tiles_str){
	H = Hin;
	n_tiles = n_tiles_in;
	unique_tiles = MatrixXi::Zero(H,1); //might need to do this

	/******* Extract the arrays from the matrix *******/
	char temp_char[unique_tiles_str.size() +1];
	strcpy(temp_char, unique_tiles_str.c_str());
	char *token;
	
	std::vector<char *> burner_array;
	token = strtok (temp_char,"[");

	while (token != NULL)
	{	
		burner_array.push_back(token);
		token = strtok (NULL, "[");
	}

	for (size_t i = 0; i < Hin; i++)
	{	
		std::vector<int> ind_num_array;
		char *token;
		token = strtok (burner_array[i],",]");
		int j = 0;
		while (token != NULL)
		{	
			token = strtok (NULL, ",]");
			std::stringstream ss;
			ss << token;
			int temp_val;
			ss >> temp_val;
			ind_num_array.push_back(temp_val);
			j++;
		}

		unique_tiles = MatrixXi::Zero(H,j);
		for (size_t k = 0; k < j; k++)
		{
			unique_tiles(i,k) = ind_num_array[k];
		}
	}
}

Hlevel::~Hlevel(){

}

void Hlevel::get_tile_data(){
	// get_history of vector<vector<>> and convert to Eigen matrix
	// 'verts_cd' built from hcost_tile_library
	// 'graph_cd from 'get_adjacency_matrix_4conn(verts_cd)
	// vector<vector<unsigned long int> > start_vertex_histories;
	// get_history(start_vertex_name, lift_H, start_vertex_histories);

}

void Hlevel::add_tile(std::shared_ptr<Tile> newTile){
	// If tile is unique, add to Tiles, increment n_tiles, and update unique_tiles
	bool is_new_tile = true;
	if (n_tiles == 0){
		unique_tiles = newTile->traversal_type;
	}else{
		// need to look through all unique_tiles cols to find match for newTile traversal_type
		for (int tile_col = 0; tile_col < unique_tiles.cols(); tile_col++){
			if (newTile->traversal_type.isApprox(unique_tiles.col(tile_col))){
				is_new_tile = false;
				break;
			}
		}
		if (is_new_tile){ // if still not in unique_tiles, resize and add new traversal_type
			unique_tiles.conservativeResize(NoChange, unique_tiles.cols()+1);
			unique_tiles.col(unique_tiles.cols()-1) = newTile->traversal_type;
		}
	}
	if (is_new_tile){
		std::cout << "is new tile" << std::endl;
		Tiles.insert({n_tiles,newTile});
		n_tiles++;
	}
}