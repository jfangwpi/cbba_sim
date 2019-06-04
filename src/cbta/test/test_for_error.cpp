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
        std::string alfa_str;
        std::string bta_str;
        std::string w_lower_str;
        std::string w_upper_str;
        std::string x_str;
        std::string w_str;
        std::string w_sol_str;
        std::string y_exit_str;
        std::string z_exit_str;
        std::string bta_smp_str;
        std::string alfa_sol_str;
        std::string alfa_smp_str;
        std::string w_smp_str;
        std::string x_smp_str;
        std::string FACE_REF_str;
        std::string VERTICES_PERMUTATION_str;
        std::string INVERSE_XFORM_str;

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

    j_tile.at(current_tile_name).at("tile_block").at("alfa").get_to(alfa_str);
    j_tile.at(current_tile_name).at("tile_block").at("bta").get_to(bta_str);			
    j_tile.at(current_tile_name).at("tile_block").at("w_lower").get_to(w_lower_str);
    j_tile.at(current_tile_name).at("tile_block").at("w_upper").get_to(w_upper_str);
    j_tile.at(current_tile_name).at("tile_block").at("x").get_to(x_str);
    j_tile.at(current_tile_name).at("tile_block").at("w").get_to(w_str);
    j_tile.at(current_tile_name).at("tile_block").at("w_sol").get_to(w_sol_str);
    j_tile.at(current_tile_name).at("tile_block").at("y_exit").get_to(y_exit_str);
    j_tile.at(current_tile_name).at("tile_block").at("z_exit").get_to(z_exit_str);			
    j_tile.at(current_tile_name).at("tile_block").at("bta_smp").get_to(bta_smp_str);
    j_tile.at(current_tile_name).at("tile_block").at("alfa_sol").get_to(alfa_sol_str);
    j_tile.at(current_tile_name).at("tile_block").at("alfa_smp").get_to(alfa_smp_str);
    j_tile.at(current_tile_name).at("tile_block").at("w_smp").get_to(w_smp_str);
    j_tile.at(current_tile_name).at("tile_block").at("x_smp").get_to(x_smp_str);	

/********** alfa_str **********/
	char temp_char_double[alfa_str.size() +1];
	strcpy(temp_char_double, alfa_str.c_str());
	char *token;
	
	std::vector<char *> burner_array;
	token = strtok (temp_char_double,"[");

	int counter = 0;
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
	MatrixXd alfa = MatrixXd::Zero(counter,n);

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
			alfa(i,k) = ind_num_array[k];
		}
	}
	
/********** bta_str **********/
	temp_char_double[bta_str.size() +1];
	strcpy(temp_char_double, bta_str.c_str());
	burner_array.clear();
	
	token = strtok (temp_char_double,"[");

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
	MatrixXd bta = MatrixXd::Zero(counter,n);
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
			bta(i,k) = ind_num_array[k];
		}
	}

/********** w_lower_str **********/
	temp_char_double[w_lower_str.size() +1];
	strcpy(temp_char_double, w_lower_str.c_str());
	burner_array.clear();
	
	token = strtok (temp_char_double,"[");

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
	MatrixXd w_lower = MatrixXd::Zero(counter,n);
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
			w_lower(i,k) = ind_num_array[k];
		}
	}

/********** w_upper_str **********/
	temp_char_double[w_upper_str.size() +1];
	strcpy(temp_char_double, w_upper_str.c_str());
	burner_array.clear();

	token = strtok (temp_char_double,"[");

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
	MatrixXd w_upper = MatrixXd::Zero(counter,n);
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
			w_upper(i,k) = ind_num_array[k];
		}
	}

/********** x_str **********/
	temp_char_double[x_str.size() +1];
	strcpy(temp_char_double, x_str.c_str());
	burner_array.clear();
	
	token = strtok (temp_char_double,"[");

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
	MatrixXd x = MatrixXd::Zero(counter,n);
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
			x(i,k) = ind_num_array[k];
		}
	}

/********** w_str **********/
	temp_char_double[w_str.size() +1];
	strcpy(temp_char_double, w_str.c_str());
	burner_array.clear();
	
	token = strtok (temp_char_double,"[");

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
	MatrixXd w = MatrixXd::Zero(counter,n);
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
			w(i,k) = ind_num_array[k];
		}
	}

/********** w_sol_str **********/
	temp_char_double[w_sol_str.size() +1];
	strcpy(temp_char_double, w_sol_str.c_str());
	burner_array.clear();
	
	token = strtok (temp_char_double,"[");

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
	MatrixXd w_sol = MatrixXd::Zero(counter,n);
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
			w_sol(i,k) = ind_num_array[k];
		}
	}

/********** y_exit_str **********/
	temp_char_double[y_exit_str.size() +1];
	strcpy(temp_char_double, y_exit_str.c_str());
	burner_array.clear();
	
	token = strtok (temp_char_double,"[");

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
	MatrixXd y_exit = MatrixXd::Zero(counter,n);
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
			y_exit(i,k) = ind_num_array[k];
		}
	}
	
/********** z_exit_str **********/
	temp_char_double[z_exit_str.size() +1];
	strcpy(temp_char_double, z_exit_str.c_str());
	burner_array.clear();
	
	token = strtok (temp_char_double,"[");

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
	MatrixXd z_exit = MatrixXd::Zero(counter,n);
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
			z_exit(i,k) = ind_num_array[k];
		}
	}

/********** bta_smp_str **********/
	temp_char_double[bta_smp_str.size() +1];
	strcpy(temp_char_double, bta_smp_str.c_str());
	burner_array.clear();
	
	token = strtok (temp_char_double,"[");

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
	MatrixXd bta_smp = MatrixXd::Zero(counter,n);
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
			bta_smp(i,k) = ind_num_array[k];
		}
	}

/********** alfa_sol_str **********/
	temp_char_double[alfa_sol_str.size() +1];
	strcpy(temp_char_double, alfa_sol_str.c_str());
	burner_array.clear();

	token = strtok (temp_char_double,"[");

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
	MatrixXd alfa_sol = MatrixXd::Zero(counter,n);
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
			alfa_sol(i,k) = ind_num_array[k];
		}
	}

/********** alfa_smp_str **********/
	temp_char_double[alfa_smp_str.size() +1];
	strcpy(temp_char_double, alfa_smp_str.c_str());
	burner_array.clear();
	
	token = strtok (temp_char_double,"[");

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
	MatrixXd alfa_smp = MatrixXd::Zero(counter,n);
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
			alfa_smp(i,k) = ind_num_array[k];
		}
	}

/********** w_smp_str **********/
	temp_char_double[w_smp_str.size() +1];
	strcpy(temp_char_double, w_smp_str.c_str());
	burner_array.clear();
	
	token = strtok (temp_char_double,"[");

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
	MatrixXd w_smp = MatrixXd::Zero(counter,n);
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
			w_smp(i,k) = ind_num_array[k];
		}
	}

/********** x_smp_str **********/
	temp_char_double[x_smp_str.size() +1];
	strcpy(temp_char_double, x_smp_str.c_str());
	burner_array.clear();
	
	token = strtok (temp_char_double,"[");

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
	MatrixXd x_smp = MatrixXd::Zero(counter,n);
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
			x_smp(i,k) = ind_num_array[k];
		}
	}
	ss.clear();

	


std::cout << "This test was sucessfull" << std::endl;
}