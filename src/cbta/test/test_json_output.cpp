// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>
#include <bitset>
#include <memory>
#include <math.h>
// opencv
#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>

// user
#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "map/square_grid.hpp"
#include "vis/graph_vis.hpp"

#include "ltl/buchi_automaton.hpp"
#include "ltl/product_automaton.hpp"
#include "cbta/graph_lift.hpp"
#include "cbta/hcost_interface.hpp"

#include "nlohmann/json.hpp"

using nlohmann::json;
	// void to_json(json& j, const TileTraversalData& data){
	// 	j = json{"Hlevels", data.Hlevels};
	// 	std::string test_string = j.dump();
	// 	std::cout << test_string << std::endl;
	// }



using namespace cv;
using namespace librav;

int main(int argc, char** argv )
{
	for(int desired_H = 1; desired_H < 4; desired_H++){
		int r_min 		= 3; // change this value in hcost_interface.cpp
		/*** Create the text file ***/
		std::ofstream file_;
		file_.open("tile_traversal_data_H" + std::to_string(desired_H) + "_R" + std::to_string(r_min) + ".txt");
		

		/*** 0. Preprocessing CBTA data ***/
		TileTraversalData tile_traversal_data = HCost::hcost_preprocessing();
		const TileTraversalData& ttd = tile_traversal_data;
		const std::shared_ptr<Hlevel> current_Hlevel = ttd.Hlevels.at(desired_H);
			
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
			j["Hlevels"]["unique_tiles"] = unique_tiles_str;
			
			// 					 // std::cout <<    current_tile->FACE_REF				<< std::endl;
			// These are private // std::cout <<    current_tile->VERTICES_PERMUTATION	<< std::endl;
			// 					 //	std::cout <<    current_tile->INVERSE_XFORM		<< std::endl;	


				
			// Tiles(n)
			std::shared_ptr<Tile> current_tile;
			std::cout << "----------" << std::endl;
			for(int n = 0; n < current_Hlevel->Tiles.size(); n++){
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
						// connectivity_str.append(",");
						connectivity_str.append(std::to_string(it.col()));
						// connectivity_str.append("],");
					}
				}
				// connectivity_str.pop_back();

				j[current_tile_name]["channel_data"] 		= channel_data_str;
				j[current_tile_name]["cell_vertices"] 		= cell_vertices_str;
				j[current_tile_name]["traversal_type"] 		= traversal_type_str;
				j[current_tile_name]["cell_xform"] 			= cell_xform_str;
				j[current_tile_name]["traversal_faces"]		= traversal_faces_str;
				j[current_tile_name]["cell_edge"] 			= cell_edge_str;
				j[current_tile_name]["connectivity"] 		= connectivity_str;

				//tile_block
					std::shared_ptr<TileBlock> current_tile_block = current_tile->tile_block;
					
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

					//H
					//y_exit	// <--- These are private //
					//z_exit	// <--- These are private //
					//bta_smp	// <--- These are private //
					//alfa_sol_str	// <--- These are private //
					//alfa_smp_str	// <--- These are private //
					//w_smp_str		// <--- These are private //
					//x_smp_str		// <--- These are private //
					//alfa
					//bta
					//w_lower
					//w_upper
					//x
					//w
					//w_sol


				

			}
				//channel_data
				//cell_vertices
				//traversal_type
				//cell_xform
				//traversal_faces
				//cell_edge
				//connectivity				/////////// need to add these to the JSON output /////////
				//FACE_REF					/////////// need to add these to the JSON output /////////
				//VERTICES_PERMUTATION		/////////// need to add these to the JSON output /////////
				//INVERSE_XFORM				/////////// need to add these to the JSON output /////////


		std::string s = j.dump();
		// std::cout << "------- JSON Output -------" << std::endl;
		// std::cout << s << std::endl;

		// Write the json string to the output file

		if(file_.is_open())
			file_ << std::setw(4) << s << std::endl;

		file_.close();
	}
	return 0;
}

