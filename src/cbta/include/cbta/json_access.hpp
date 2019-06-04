/*
 *  json_access.hpp
 *
 *  namespace definition for reading and writing 
 *  JSON file containing CBTA traversal data
 * 
 *  Created on: May 30th, 2019
 *      Author: Jack O'Neill
 *      Questions/comments: (203) 558-2221
 */

#ifndef JSON_ACCESS_HPP
#define JSON_ACCESS_HPP

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


namespace jsonReadWrite{
    void get_to_json(const int desired_H_begin, const int desired_H_end){
    	for(int curr_H = desired_H_begin; curr_H < desired_H_end + 1; curr_H++){
			int r_min 		= 3; // change this value in hcost_interface.cpp
			/*** Create the text file ***/
			std::ofstream Hlevel_file_;
			Hlevel_file_.open("ttd_H" + std::to_string(curr_H) + "_R" + std::to_string(r_min) + ".txt");
			

			/*** 0. Preprocessing CBTA data ***/
			librav::TileTraversalData tile_traversal_data = librav::HCost::hcost_preprocessing();
			const librav::TileTraversalData& ttd = tile_traversal_data;
			const std::shared_ptr<librav::Hlevel> current_Hlevel = ttd.Hlevels.at(curr_H);
				
			using json = nlohmann::json;
			
			json j;
			// TileTraversalData
			j["N_REGION_W"] = tile_traversal_data.N_REGION_W;
			j["N_REGION_PSI"] = tile_traversal_data.N_REGION_PSI;
			j["N_REGION_SPD"] = tile_traversal_data.N_REGION_SPD;
			j["N_REGION_TOTAL"] = tile_traversal_data.N_REGION_TOTAL;
			
			// REGION_BD
				j["REGION_BD"]["region_w_lower"] = ttd.region_bd.region_w_lower;
				j["REGION_BD"]["region_w_upper"] = ttd.region_bd.region_w_upper;
				j["REGION_BD"]["region_psi_lower"] = ttd.region_bd.region_psi_lower;
				j["REGION_BD"]["region_psi_upper"] = ttd.region_bd.region_psi_upper;
				j["REGION_BD"]["region_vel_lower"] = ttd.region_bd.region_vel_lower;
				j["REGION_BD"]["region_vel_upper"] = ttd.region_bd.region_vel_upper;

			// Hlevels				
				std::string unique_tiles_str;
				for(int i=0;i<current_Hlevel->unique_tiles.rows(); i++){
					unique_tiles_str.append("[");
					for(int j=0;j<current_Hlevel->unique_tiles.cols(); j++){
						unique_tiles_str.append(std::to_string(current_Hlevel->unique_tiles(i,j)));
						unique_tiles_str.append(",");
					}
					unique_tiles_str.pop_back();
					unique_tiles_str.append("],");
				}
				unique_tiles_str.pop_back();

				j["Hlevels"]["H"] 						= current_Hlevel->H;
				j["Hlevels"]["n_tiles"] 				= current_Hlevel->n_tiles;	
				j["Hlevels"]["unique_tiles"] 			= unique_tiles_str;
				
				// 					 // std::cout <<    current_tile->FACE_REF				<< std::endl;
				// These are private // std::cout <<    current_tile->VERTICES_PERMUTATION	<< std::endl;
				// 					 //	std::cout <<    current_tile->INVERSE_XFORM		<< std::endl;	

			//*** Write to the Hlevel file ***//
			std::string s = j.dump();
			if(Hlevel_file_.is_open())
				Hlevel_file_ << s << std::endl;
			Hlevel_file_.close();

			// Tiles(n)
				std::shared_ptr<librav::Tile> current_tile;
				std::cout << "----------" << std::endl;
				for(int n = 0; n < current_Hlevel->Tiles.size(); n++){
					//*** Create a tile file for current Hlevel ***//
					std::ofstream tile_file_;
					tile_file_.open("H" + std::to_string(curr_H) + "_R" + std::to_string(r_min) + "_Tile" + std::to_string(n) +".txt");
					j.clear();

					//*** Start going through all tile data ***//
					current_tile = current_Hlevel->Tiles.at(n);
					std::string current_tile_name = "Tile_" + std::to_string(n);
					std::cout << current_tile_name << std::endl;
					std::cout << "----------" << std::endl;

					std::string channel_data_str;
					for(int i=0;i<current_tile->channel_data.rows(); i++){
						channel_data_str.append("[");
						for(int j=0;j<current_tile->channel_data.cols(); j++){
							channel_data_str.append(std::to_string(current_tile->channel_data(i,j)));
							channel_data_str.append(",");
						}
						channel_data_str.pop_back();
						channel_data_str.append("],");
					}
					channel_data_str.pop_back();

					std::string cell_vertices_str;
					for(int i=0;i<current_tile->cell_vertices.rows(); i++){
						cell_vertices_str.append("[");
						for(int j=0;j<current_tile->cell_vertices.cols(); j++){
							cell_vertices_str.append(std::to_string(current_tile->cell_vertices(i,j)));
							cell_vertices_str.append(",");
						}
						cell_vertices_str.pop_back();
						cell_vertices_str.append("],");
					}
					cell_vertices_str.pop_back();

					std::string traversal_type_str;
					for(int i=0;i<current_tile->traversal_type.rows(); i++){
						traversal_type_str.append("[");
						for(int j=0;j<current_tile->traversal_type.cols(); j++){
							traversal_type_str.append(std::to_string(current_tile->traversal_type(i,j)));
							traversal_type_str.append(",");
						}
						traversal_type_str.pop_back();
						traversal_type_str.append("],");
					}
					traversal_type_str.pop_back();

					std::string cell_xform_str;
					for(int i=0;i<current_tile->cell_xform.rows(); i++){
						cell_xform_str.append("[");
						for(int j=0;j<current_tile->cell_xform.cols(); j++){
							cell_xform_str.append(std::to_string(current_tile->cell_xform(i,j)));
							cell_xform_str.append(",");
						}
						cell_xform_str.pop_back();
						cell_xform_str.append("],");
					}
					cell_xform_str.pop_back();

					std::string traversal_faces_str;
					for(int i=0;i<current_tile->traversal_faces.rows(); i++){
						traversal_faces_str.append("[");
						for(int j=0;j<current_tile->traversal_faces.cols(); j++){
							traversal_faces_str.append(std::to_string(current_tile->traversal_faces(i,j)));
							traversal_faces_str.append(",");
						}
						traversal_faces_str.pop_back();
						traversal_faces_str.append("],");
					}
					traversal_faces_str.pop_back();

					std::string cell_edge_str;
					for(int i=0;i<current_tile->cell_edge.rows(); i++){
						cell_edge_str.append("[");
						for(int j=0;j<current_tile->cell_edge.cols(); j++){
							cell_edge_str.append(std::to_string(current_tile->cell_edge(i,j)));
							cell_edge_str.append(",");
						}
						cell_edge_str.pop_back();
						cell_edge_str.append("],");
					}
					cell_edge_str.pop_back();

					/* 																 * 
					* The computer seems to run out of RAM 						 * 						
					* in this part of the code below. I was unable to 				 * 		
					* determine a solution to this problem. It very well			 * 			
					* might be solved by running it on a computer with more RAM.	 * 					
					* 																 * 
					* For now, we only have access to Hlevels 1-3					 * 	
					*																 */

					std::string connectivity_str;
					auto con_mat = *(current_tile->connectivity.get());

					for (int k = 0; k < con_mat.outerSize(); ++k)
					{	
						for (Eigen::SparseMatrix<int, 1>::InnerIterator it(con_mat, k); it; ++it)
						{
							// connectivity_str.append("[");
							connectivity_str.append(std::to_string(it.row()));
							connectivity_str.append(",");
							connectivity_str.append(std::to_string(it.col()));
							connectivity_str.append(";");
						}
					}
					connectivity_str.pop_back();

					j[current_tile_name]["channel_data"] 		= channel_data_str;
					j[current_tile_name]["cell_vertices"] 		= cell_vertices_str;
					j[current_tile_name]["traversal_type"] 		= traversal_type_str;
					j[current_tile_name]["cell_xform"] 			= cell_xform_str;
					j[current_tile_name]["traversal_faces"]		= traversal_faces_str;
					j[current_tile_name]["cell_edge"] 			= cell_edge_str;
					j[current_tile_name]["connectivity"] 		= connectivity_str;

					//tile_block
						std::shared_ptr<librav::TileBlock> current_tile_block = current_tile->tile_block;
						
						std::string alfa_str;
						for(int i=0;i<current_tile_block->alfa.rows(); i++){
							alfa_str.append("[");
							for(int j=0;j<current_tile_block->alfa.cols(); j++){
								alfa_str.append(std::to_string(current_tile_block->alfa(i,j)));
								alfa_str.append(",");
							}
							alfa_str.pop_back();
							alfa_str.append("],");
						}
						alfa_str.pop_back();

						std::string bta_str;
						for(int i=0;i<current_tile_block->bta.rows(); i++){
							bta_str.append("[");
							for(int j=0;j<current_tile_block->bta.cols(); j++){
								bta_str.append(std::to_string(current_tile_block->bta(i,j)));
								bta_str.append(",");
							}
							bta_str.pop_back();
							bta_str.append("],");
						}
						bta_str.pop_back();

						std::string w_lower_str;
						for(int i=0;i<current_tile_block->w_lower.rows(); i++){
							w_lower_str.append("[");
							for(int j=0;j<current_tile_block->w_lower.cols(); j++){
								w_lower_str.append(std::to_string(current_tile_block->w_lower(i,j)));
								w_lower_str.append(",");
							}
							w_lower_str.pop_back();
							w_lower_str.append("],");
						}
						w_lower_str.pop_back();

						std::string w_upper_str;
						for(int i=0;i<current_tile_block->w_upper.rows(); i++){
							w_upper_str.append("[");
							for(int j=0;j<current_tile_block->w_upper.cols(); j++){
								w_upper_str.append(std::to_string(current_tile_block->w_upper(i,j)));
								w_upper_str.append(",");
							}
							w_upper_str.pop_back();
							w_upper_str.append("],");
						}
						w_upper_str.pop_back();

						std::string x_str;
						for(int i=0;i<current_tile_block->x.rows(); i++){
							x_str.append("[");
							for(int j=0;j<current_tile_block->x.cols(); j++){
								x_str.append(std::to_string(current_tile_block->x(i,j)));
								x_str.append(",");
							}
							x_str.pop_back();
							x_str.append("],");
						}
						x_str.pop_back();

						std::string w_str;
						for(int i=0;i<current_tile_block->w.rows(); i++){
							w_str.append("[");
							for(int j=0;j<current_tile_block->w.cols(); j++){
								w_str.append(std::to_string(current_tile_block->w(i,j)));
								w_str.append(",");
							}
							w_str.pop_back();
							w_str.append("],");
						}
						w_str.pop_back();

						std::string w_sol_str;
						for(int i=0;i<current_tile_block->w_sol.rows(); i++){
							w_sol_str.append("[");
							for(int j=0;j<current_tile_block->w_sol.cols(); j++){
								w_sol_str.append(std::to_string(current_tile_block->w_sol(i,j)));
								w_sol_str.append(",");
							}
							w_sol_str.pop_back();
							w_sol_str.append("],");
						}
						w_sol_str.pop_back();


						j[current_tile_name]["tile_block"]["alfa"]		= alfa_str;
						j[current_tile_name]["tile_block"]["bta"]		= bta_str;
						j[current_tile_name]["tile_block"]["w_lower"]	= w_lower_str;
						j[current_tile_name]["tile_block"]["w_upper"]	= w_upper_str;
						j[current_tile_name]["tile_block"]["x"]			= x_str;
						j[current_tile_name]["tile_block"]["w"]			= w_str;
						j[current_tile_name]["tile_block"]["w_sol"]		= w_sol_str;


						// ********* Previously private variables ********** //

						std::string y_exit_str;
						for(int i=0;i<current_tile_block->y_exit.rows(); i++){
							y_exit_str.append("[");
							for(int j=0;j<current_tile_block->y_exit.cols(); j++){
								y_exit_str.append(std::to_string(current_tile_block->y_exit(i,j)));
								y_exit_str.append(",");
							}
							y_exit_str.pop_back();
							y_exit_str.append("],");
						}
						y_exit_str.pop_back();

						std::string z_exit_str;
						for(int i=0;i<current_tile_block->z_exit.rows(); i++){
							z_exit_str.append("[");
							for(int j=0;j<current_tile_block->z_exit.cols(); j++){
								z_exit_str.append(std::to_string(current_tile_block->z_exit(i,j)));
								z_exit_str.append(",");
							}
							z_exit_str.pop_back();
							z_exit_str.append("],");
						}
						z_exit_str.pop_back();

						std::string bta_smp_str;
						for(int i=0;i<current_tile_block->bta_smp.rows(); i++){
							bta_smp_str.append("[");
							for(int j=0;j<current_tile_block->bta_smp.cols(); j++){
								bta_smp_str.append(std::to_string(current_tile_block->bta_smp(i,j)));
								bta_smp_str.append(",");
							}
							bta_smp_str.pop_back();
							bta_smp_str.append("],");
						}
						bta_smp_str.pop_back();

						std::string alfa_sol_str;
						for(int i=0;i<current_tile_block->alfa_sol.rows(); i++){
							alfa_sol_str.append("[");
							for(int j=0;j<current_tile_block->alfa_sol.cols(); j++){
								alfa_sol_str.append(std::to_string(current_tile_block->alfa_sol(i,j)));
								alfa_sol_str.append(",");
							}
							alfa_sol_str.pop_back();
							alfa_sol_str.append("],");
						}
						alfa_sol_str.pop_back();

						std::string alfa_smp_str;
						for(int i=0;i<current_tile_block->alfa_smp.rows(); i++){
							alfa_smp_str.append("[");
							for(int j=0;j<current_tile_block->alfa_smp.cols(); j++){
								alfa_smp_str.append(std::to_string(current_tile_block->alfa_smp(i,j)));
								alfa_smp_str.append(",");
							}
							alfa_smp_str.pop_back();
							alfa_smp_str.append("],");
						}
						alfa_smp_str.pop_back();

						std::string w_smp_str;
						for(int i=0;i<current_tile_block->w_smp.rows(); i++){
							w_smp_str.append("[");
							for(int j=0;j<current_tile_block->w_smp.cols(); j++){
								w_smp_str.append(std::to_string(current_tile_block->w_smp(i,j)));
								w_smp_str.append(",");
							}
							w_smp_str.pop_back();
							w_smp_str.append("],");
						}
						w_smp_str.pop_back();

						std::string x_smp_str;
						for(int i=0;i<current_tile_block->x_smp.rows(); i++){
							x_smp_str.append("[");
							for(int j=0;j<current_tile_block->x_smp.cols(); j++){
								x_smp_str.append(std::to_string(current_tile_block->x_smp(i,j)));
								x_smp_str.append(",");
							}
							x_smp_str.pop_back();
							x_smp_str.append("],");
						}
						x_smp_str.pop_back();


						j[current_tile_name]["tile_block"]["y_exit"]		= y_exit_str;
						j[current_tile_name]["tile_block"]["z_exit"]		= z_exit_str;
						j[current_tile_name]["tile_block"]["bta_smp"]		= bta_smp_str;
						j[current_tile_name]["tile_block"]["alfa_sol"]		= alfa_sol_str;
						j[current_tile_name]["tile_block"]["alfa_smp"]		= alfa_smp_str;
						j[current_tile_name]["tile_block"]["w_smp"]			= w_smp_str;
						j[current_tile_name]["tile_block"]["x_smp"]			= x_smp_str;

						//*** Write to the Tile File ***//
						std::string s = j.dump();
						if(tile_file_.is_open())
							tile_file_ << s << std::endl;
						tile_file_.close();
				}
			}
    }
    void get_from_json(const int rmin, librav::TileTraversalData& ttd, int curr_H){	

		std::string folder = "./H"+std::to_string(curr_H)+"_R"+std::to_string(rmin)+"/";

        using nlohmann::json;
		std::ifstream i(folder+"ttd_H"+std::to_string(curr_H)+"_R"+std::to_string(rmin)+".txt");
		nlohmann::json j_Hlevel;
		i >> j_Hlevel;
        /********** N_REGION **********/
        j_Hlevel.at("N_REGION_W").get_to(ttd.N_REGION_W);
        j_Hlevel.at("N_REGION_PSI").get_to(ttd.N_REGION_PSI);
        j_Hlevel.at("N_REGION_SPD").get_to(ttd.N_REGION_SPD);
        j_Hlevel.at("N_REGION_TOTAL").get_to(ttd.N_REGION_TOTAL);
        /********** region_bd **********/
        j_Hlevel.at("REGION_BD").at("region_psi_lower").get_to(ttd.region_bd.region_psi_lower);
        j_Hlevel.at("REGION_BD").at("region_psi_upper").get_to(ttd.region_bd.region_psi_upper);
        j_Hlevel.at("REGION_BD").at("region_w_lower").get_to(ttd.region_bd.region_w_lower);	
        j_Hlevel.at("REGION_BD").at("region_w_upper").get_to(ttd.region_bd.region_w_upper);	
        j_Hlevel.at("REGION_BD").at("region_vel_lower").get_to(ttd.region_bd.region_vel_lower);	
        j_Hlevel.at("REGION_BD").at("region_vel_upper").get_to(ttd.region_bd.region_vel_upper);
        
        /********** Hlevels **********/
        unsigned int H;
        unsigned int n_tiles;
        Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic> unique_tiles;
        j_Hlevel.at("Hlevels").at("H").get_to(H);
        j_Hlevel.at("Hlevels").at("n_tiles").get_to(n_tiles);
        std::string unique_tiles_str;
        j_Hlevel.at("Hlevels").at("unique_tiles").get_to(unique_tiles_str);
        std::shared_ptr<librav::Hlevel> current_Hlevel = std::make_shared<librav::Hlevel>(H,n_tiles,unique_tiles_str);
        /********** Tiles **********/
        std::string cell_edge_str;
        std::string cell_vertices_str;
        std::string cell_xform_str;
        std::string channel_data_str;
        std::string connectivity_str;
        std::string traversal_type_str;
        std::string traversal_faces_str;
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
		std::cout << "Probe" <<std::endl;
        for(int n = 0; n < n_tiles; n++){
			//*** Define the file name to be read ***//
			std::ifstream i_temp(folder+"H"+std::to_string(curr_H)+"_R"+std::to_string(rmin)+"_Tile"+std::to_string(n)+".txt");
			nlohmann::json j_tile;
			i_temp >> j_tile;

            std::stringstream str;
            str << "Tile_" << n;
            std::string title = str.str();
            const char *current_tile_name = title.c_str();
            std::cout << str.str() << std::endl;


            j_tile.at(current_tile_name).at("traversal_type").get_to(traversal_type_str);
            j_tile.at(current_tile_name).at("traversal_faces").get_to(traversal_faces_str);			
            j_tile.at(current_tile_name).at("cell_edge").get_to(cell_edge_str);
            j_tile.at(current_tile_name).at("cell_vertices").get_to(cell_vertices_str);
            j_tile.at(current_tile_name).at("cell_xform").get_to(cell_xform_str);
            j_tile.at(current_tile_name).at("channel_data").get_to(channel_data_str);
            j_tile.at(current_tile_name).at("connectivity").get_to(connectivity_str);
            std::shared_ptr<librav::Tile> temp_tile = std::make_shared<librav::Tile>(traversal_type_str,
                                                                                    traversal_faces_str,
                                                                                    cell_xform_str,
                                                                                    channel_data_str,
                                                                                    cell_edge_str,
                                                                                    cell_vertices_str,
                                                                                    connectivity_str);
                                                                                    //  FACE_REF_str,
                                                                                    //  VERTICES_PERMUTATION_str,
                                                                                    //  INVERSE_XFORM_str);
            
            /********** Tile Block **********/
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
            std::shared_ptr<librav::TileBlock> temp_tile_block = std::make_shared<librav::TileBlock>
            (ttd.region_bd, temp_tile, curr_H,
            alfa_str,
            bta_str,
            w_lower_str,
            w_upper_str,
            x_str,
            w_str,
            w_sol_str,
            
            y_exit_str,
            z_exit_str,
            bta_smp_str,
            alfa_sol_str,
            alfa_smp_str,
            w_smp_str,
            x_smp_str);
            
            temp_tile->addTileBlock(ttd.region_bd,temp_tile_block,curr_H);
            //std::cout << n << std::endl;
            // std::cout << std::endl << temp_tile->traversal_type << std::endl << std::endl;
            // std::cout << current_Hlevel->unique_tiles << std::endl << std::endl;
            // std::cout << temp_tile->cell_vertices << std::endl << std::endl;
            // getchar();
            current_Hlevel->Tiles.insert({n,temp_tile});
            // current_Hlevel->add_tile(temp_tile);
        }
        ttd.Hlevels.insert({curr_H,current_Hlevel});
        // getchar();
        }
}


#endif /* JSON_ACCESS_HPP */