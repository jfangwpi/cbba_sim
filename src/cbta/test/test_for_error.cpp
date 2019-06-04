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
        std::string cell_edge_str;
        std::string cell_vertices_str;
        std::string cell_xform_str;
        std::string channel_data_str;
        std::string traversal_type_str;
        std::string traversal_faces_str;

//** Gather data from json **//
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
    j_tile.at(current_tile_name).at("traversal_type").get_to(traversal_type_str);
    j_tile.at(current_tile_name).at("traversal_faces").get_to(traversal_faces_str);			
    j_tile.at(current_tile_name).at("cell_edge").get_to(cell_edge_str);
    j_tile.at(current_tile_name).at("cell_vertices").get_to(cell_vertices_str);
    j_tile.at(current_tile_name).at("cell_xform").get_to(cell_xform_str);
    j_tile.at(current_tile_name).at("channel_data").get_to(channel_data_str);

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

/******* cell_xform_str *******/
	char temp_char[cell_xform_str.size() +1];
	strcpy(temp_char, cell_xform_str.c_str());
	burner_array.clear();
	token = strtok (temp_char,"[");

	counter = 0;
	while (token != NULL)
	{	
		burner_array.push_back(token);
		token = strtok (NULL, "[");
		counter++;
	}
	

	std::stringstream ss;
	ss << burner_array[0];
	std::string test_str;
	ss >> test_str;
	size_t n = std::count(test_str.begin(), test_str.end(), ',');
	Eigen::MatrixXi cell_xform;
	cell_xform = MatrixXi::Zero(counter,n);
	test_str.clear();

	for (size_t i = 0; i < counter; i++)
	{	
		std::vector<int> ind_num_array;
		char *token;
		token = strtok (burner_array[i],",]");
		int j = 0;
		std::stringstream ss;
		while (token != NULL)
		{
			ss.clear();	
			ss << token;
			int temp_val;
			ss >> temp_val;
			ind_num_array.push_back(temp_val);
			token = strtok (NULL, ",]");
			j++;
		}

		for (size_t k = 0; k < j; k++)
		{
			cell_xform(i,k) = ind_num_array[k];
		}
	}
	
/******* channel_data_str *******/
	temp_char[channel_data_str.size() +1];
	strcpy(temp_char, channel_data_str.c_str());
	
	burner_array.clear();
	token = strtok (temp_char,"[");

	counter = 0;
	while (token != NULL)
	{	
		burner_array.push_back(token);
		token = strtok (NULL, "[");
		counter++;
	}

	ss.clear();
	ss << burner_array[0];
	ss >> test_str;
	n = std::count(test_str.begin(), test_str.end(), ',');
	Eigen::MatrixXi  channel_data;
	channel_data = MatrixXi::Zero(counter,n);
	test_str.clear();


	for (size_t i = 0; i < counter; i++)
	{	
		std::vector<int> ind_num_array;
		char *token;
		token = strtok(burner_array[i],",]");
		int j = 0;
		std::stringstream ss;
		while (token != NULL)
		{
			ss.clear();	
			ss << token;
			int temp_val;
			ss >> temp_val;
			ind_num_array.push_back(temp_val);
			token = strtok (NULL, ",]");
			j++;
		}

		for (size_t k = 0; k < j; k++)
		{
			channel_data(i,k) = ind_num_array[k];
		}
	}
/******* traversal_faces *******/
	temp_char[traversal_faces_str.size() +1];
	strcpy(temp_char, traversal_faces_str.c_str());
	
	burner_array.clear();
	token = strtok (temp_char,"[");

	counter = 0;
	while (token != NULL)
	{	
		burner_array.push_back(token);
		token = strtok (NULL, "[");
		counter++;
	}

	ss.clear();
	ss << burner_array[0];
	
	ss >> test_str;
	n = std::count(test_str.begin(), test_str.end(), ',');
	Eigen::MatrixXi traversal_faces;
	traversal_faces = MatrixXi::Zero(counter,n);
	test_str.clear();


	for (size_t i = 0; i < counter; i++)
	{	
		std::vector<int> ind_num_array;
		char *token;
		token = strtok(burner_array[i],",]");
		int j = 0;
		std::stringstream ss;
		while (token != NULL)
		{
			ss.clear();	
			ss << token;
			int temp_val;
			ss >> temp_val;
			ind_num_array.push_back(temp_val);
			token = strtok (NULL, ",]");
			j++;
		}

		for (size_t k = 0; k < j; k++)
		{
			traversal_faces(i,k) = ind_num_array[k];
		}
	}
/******* traversal_type *******/
	temp_char[traversal_type_str.size() +1];
	strcpy(temp_char, traversal_type_str.c_str());
	
	burner_array.clear();
	token = strtok (temp_char,"[");

	counter = 0;
	while (token != NULL)
	{	
		burner_array.push_back(token);
		token = strtok (NULL, "[");
		counter++;
	}

	ss.clear();
	ss << burner_array[0];
	ss >> test_str;
	n = std::count(test_str.begin(), test_str.end(), ',');
	Eigen::MatrixXi traversal_type;
	traversal_type = MatrixXi::Zero(counter,n);
	test_str.clear();


	for (size_t i = 0; i < counter; i++)
	{	
		std::vector<int> ind_num_array;
		char *token;
		token = strtok(burner_array[i],",]");
		int j = 0;
		std::stringstream ss;
		while (token != NULL)
		{
			ss.clear();	
			ss << token;
			int temp_val;
			ss >> temp_val;
			ind_num_array.push_back(temp_val);
			token = strtok (NULL, ",]");
			j++;
		}

		for (size_t k = 0; k < j; k++)
		{
			traversal_type(i,k) = ind_num_array[k];
			// std::cout << traversal_type(i,k) << std::endl;
		}
	}
	
/******* cell_edge_str *******/
	char temp_char_double[cell_edge_str.size() +1];
	strcpy(temp_char_double, cell_edge_str.c_str());
	
	burner_array.clear();

	char *token_double = strtok (temp_char_double,"[");

	counter = 0;
	while (token_double != NULL)
	{	
		burner_array.push_back(token_double);
		token_double = strtok (NULL, "[");
		counter++;
	}

	ss.clear();
	ss << burner_array[0];
	ss >> test_str;
	n = std::count(test_str.begin(), test_str.end(), ',');
	Eigen::MatrixXd cell_edge;
	cell_edge = MatrixXd::Zero(counter,n);
	test_str.clear();

	for (size_t i = 0; i < counter; i++)
	{	
		std::vector<double> ind_num_array;
		char *token_double;
		token_double = strtok (burner_array[i],",]");
		int j = 0;
		std::stringstream ss;
		while (token_double != NULL)
		{
			ss.clear();	
			ss << token_double;
			double temp_val;
			ss >> temp_val;
			ind_num_array.push_back(temp_val);
			token_double = strtok (NULL, ",]");
			j++;
		}

		for (size_t k = 0; k < j; k++)
		{
			cell_edge(i,k) = ind_num_array[k];
			// std::cout << cell_edge(i,k) << std::endl;
			
		}
	}
	
/******* cell_vertices_str *******/
	temp_char_double[cell_vertices_str.size() +1];
	strcpy(temp_char_double, cell_vertices_str.c_str());
	
	burner_array.clear();
	token_double = strtok (temp_char_double,"[");

	counter = 0;
	while (token_double != NULL)
	{	
		burner_array.push_back(token_double);
		token_double = strtok (NULL, "[");
		counter++;
	}

	ss.clear();
	ss << burner_array[0];
	
	ss >> test_str;
	n = std::count(test_str.begin(), test_str.end(), ',');
	Eigen::MatrixXd cell_vertices;
	cell_vertices = MatrixXd::Zero(counter,n);
	test_str.clear();
	ss.clear();


	for (size_t i = 0; i < counter; i++)
	{	
		std::vector<double> ind_num_array;
		char *token_double;
		token_double = strtok (burner_array[i],",]");
		int j = 0;
		std::stringstream ss;
		while (token_double != NULL)
		{
			ss.clear();	
			ss << token_double;
			double temp_val;
			ss >> temp_val;
			ind_num_array.push_back(temp_val);
			token_double = strtok (NULL, ",]");
			j++;
		}

		for (size_t k = 0; k < j; k++)
		{
			cell_vertices(i,k) = ind_num_array[k];
			// std::cout << cell_vertices(i,k) << std::endl;
		}
	}
}
