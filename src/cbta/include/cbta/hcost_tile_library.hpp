/*
 * hcost_tile_library.h
 *
 *  Created on: Feb 23, 2016
 *      Author: bscooper
 */

#ifndef HCOST_TILE_LIBRARY_HPP
#define HCOST_TILE_LIBRARY_HPP

#include <iostream>
#include <list>
#include <vector>
#include <cmath>
#include <map>
#include <algorithm>
#include <iterator>
#include <memory>
#include <functional>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SparseCore>
//#include <eigen3/Eigen/Core>
//#include <Eigen/Sparse>

namespace librav {
const double PI = 3.141592653589793;
const double T_INFINITY = 1.0e9;
const double TOL = 1.0e-9;
// Largest value of H considered
const unsigned short int MAX_H = 5;
// Minimum and Maximum speed considered
const unsigned int SPD_MIN = 1;
const unsigned int SPD_MAX = 1;

const int N_CBTA_W_SOL = 51;
const int N_CBTA_W = 101;
const double r_min = 3;

struct REGION_BD{
		std::vector<double> region_w_lower;    //REGION_BD(0,:)
		std::vector<double> region_w_upper;    //REGION_BD(1,:)
		std::vector<double> region_psi_lower;  //REGION_BD(2,:)
		std::vector<double> region_psi_upper;  //REGION_BD(3,:)
		std::vector<double> region_vel_lower;  //REGION_BD(4,:)
		std::vector<double> region_vel_upper;  //REGION_BD(5,:)
	};


class TileBlock; // say TileBlock exists without defining it, forward declaration
				 // so that Tile can declare it's instance of a TileBlock
// =============================== Tile ===================================
/* Each Tile with channel_data, cell_vertices, traversal_type,
 * cell_xform, traversal_faces, cell_edges, and connectivity
 */
class Tile{
public:
	Tile(int H, Eigen::Matrix<int,Eigen::Dynamic,4> tile_vertices);
	Tile(std::string traversal_type_str,
		   std::string traversal_faces_str,
		   std::string cell_xform_str,
		   std::string channel_data_str,
		   std::string cell_edge_str,
		   std::string cell_vertices_str,
		   std::string current_tile_name,
		   std::string file_prefix);
		//  std::string FACE_REF_str,
		//  std::string VERTICES_PERMUTATION_str,
		//  std::string INVERSE_XFORM_str);
	~Tile();

private:
	// Tile class Transition and transformation references
	Eigen::Matrix<int,12,5> FACE_REF;
	Eigen::Matrix<int,5,4> VERTICES_PERMUTATION;
	//VectorXi INVERSE_XFORM(5);
	Eigen::Matrix<int,5,1> INVERSE_XFORM;

public:

	Eigen::Matrix<int,Eigen::Dynamic,4> channel_data;
	Eigen::Matrix<double,4,Eigen::Dynamic> cell_vertices;
	Eigen::Matrix<int,Eigen::Dynamic,1> traversal_type;
	Eigen::Matrix<int,Eigen::Dynamic,2> cell_xform;
	Eigen::Matrix<int,Eigen::Dynamic,1> traversal_faces;
	Eigen::Matrix<double,Eigen::Dynamic,4> cell_edge;

	std::shared_ptr<TileBlock> tile_block;

	std::shared_ptr<Eigen::SparseMatrix<int,Eigen::RowMajor>> connectivity;

	void set_tile_data(int, Eigen::Matrix<int,Eigen::Dynamic,4>);

	void addTileBlock(REGION_BD &region_bd, std::shared_ptr<TileBlock> this_tile, int H);

	void setMatricesFromJSON(std::string traversal_type_str,
							 std::string traversal_faces_str,
							 std::string cell_xform_str,
							 std::string channel_data_str,
							 std::string cell_edge_str,
							 std::string cell_vertices_str);


	void setConnectivityFromJSON(std::string current_tile_name, std::string file_prefix);

};

// =============================== TileBlock ==============================
/* A TileBlock is associated with a Tile. TileBlock is responsible for the
 * Curvature Bounded Traversability Analysis (CBTA), and this information
 * is used in the edge costs calculations needed by the Lifted Graph
 * transition function. TileBlock needs access to it's associated Tile's
 * data: channel_data, cell_vertices, ...
 */
class TileBlock{
public:
	TileBlock();
	TileBlock(REGION_BD &region_bd, std::shared_ptr<Tile> linked_tile, int Hin);
	TileBlock(REGION_BD &REGION_BD, std::shared_ptr<Tile> linked_tile, int Hin,
					 	std::string alfa_str,
						std::string bta_str,
						std::string w_lower_str,
						std::string w_upper_str,
						std::string x_str,
						std::string w_str,
						std::string w_sol_str,
						
						std::string y_exit_str,
						std::string z_exit_str,
						std::string bta_smp_str,
						std::string alfa_sol_str,
						std::string alfa_smp_str,
						std::string w_smp_str,
						std::string x_smp_str);
	~TileBlock();
public: // private: <---------------------------------------------------******$*$*$*$*$*$*$
	int H;
	Eigen::Matrix<double,Eigen::Dynamic,1> y_exit;
	Eigen::Matrix<double,Eigen::Dynamic,1> z_exit;

	Eigen::Matrix<double,Eigen::Dynamic,N_CBTA_W> bta_smp; //Matrix<double,Dynamic,N_CBTA_W>

	Eigen::Matrix<double,Eigen::Dynamic,N_CBTA_W_SOL> alfa_sol;
	Eigen::Matrix<double,Eigen::Dynamic,N_CBTA_W> alfa_smp;

	Eigen::Matrix<double,Eigen::Dynamic,N_CBTA_W> w_smp;
	Eigen::Matrix<double,Eigen::Dynamic,N_CBTA_W> x_smp;
public:
	Eigen::Matrix<double,Eigen::Dynamic,N_CBTA_W> alfa;
	Eigen::Matrix<double,Eigen::Dynamic,N_CBTA_W> bta;
	Eigen::Matrix<double,Eigen::Dynamic,1> w_lower;
	Eigen::Matrix<double,Eigen::Dynamic,1> w_upper;
	Eigen::Matrix<double,Eigen::Dynamic,N_CBTA_W> x;
	Eigen::Matrix<double,Eigen::Dynamic,N_CBTA_W> w;
	Eigen::Matrix<double,Eigen::Dynamic,N_CBTA_W_SOL> w_sol;


	std::shared_ptr<Tile> tile;



public:
	void cbta();
	void cbra(int idx_r_from,REGION_BD &theRegion,double r_min,
				long int N_REGION_TOTAL, Eigen::RowVectorXi& region_target_2,
	//			Matrix<int,1,Dynamic>& returnMatrix);
				Eigen::RowVectorXi& region_neighbors);
	static int find_sample(Eigen::RowVectorXd& ySmp, double y);

private:
	void cbta_s1(double w, double d,
			     //Matrix<double,1,N_CBTA_W>& xSmp,
			Eigen::RowVectorXd& xSmp,
			Eigen::Matrix<double,2,N_CBTA_W>& btaSmp,
			Eigen::Matrix<double,2,1>& returnMatrix);

	void cbta_s2(double w, double d,
			     //Matrix<double,1,N_CBTA_W>& xSmp,
			Eigen::RowVectorXd& xSmp,
			Eigen::Matrix<double,2,N_CBTA_W>& btaSmp,
			Eigen::Matrix<double,2,1>& returnMatrix);

	void interp_broken_seg(Eigen::RowVectorXd& x_data,
			Eigen::Matrix<double,2,Eigen::Dynamic>& y_data,
			Eigen::RowVectorXd& x_interp,
			Eigen::Matrix<double,2,Eigen::Dynamic>& y_interp);
	template <typename Derived1, typename Derived2>
	void remove_inf_values(Eigen::MatrixBase<Derived1>& v,
			Eigen::MatrixBase<Derived2>& returnMatrix);

//	int find_sample(Matrix<double,1,N_CBTA_W>& ySmp, double y);


	//double sign_func(double x);
	template <typename Derived_a, typename Derived_b>
	void find_zeros(Eigen::MatrixBase<Derived_a>& fSmp,
			Eigen::MatrixBase<Derived_b>& returnMatrix);
//	void find_zeros(Matrix<double,1,Dynamic>& fSmp,
//			Matrix<int,1,Dynamic>& returnMatrix);

	double pi2pi(double x);

};



// ================================ Hlevel =================================
/* HcostTileLibrary contains Hlevel's H1, H2, ....
 * Each Hlevel contains a map of Tiles with channel_data, cell_vertices, traversal_type,
 * cell_xform, traversal_faces, cell_edges, and connectivity
 */
class Hlevel{
public:
	Hlevel(unsigned int Hin);
	Hlevel(unsigned int Hin, unsigned int n_tiles_in, std::string unique_tiles_str);
	~Hlevel();

public:
	unsigned int H;
	std::map<unsigned int, std::shared_ptr<Tile>> Tiles; // tiles associated with particular Hlevel
	void get_tile_data(void);
	void add_tile(std::shared_ptr<Tile> newTile);

	unsigned int n_tiles;
	Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic> unique_tiles;


};

struct TileTraversalData{
	std::map<unsigned int,std::shared_ptr<Hlevel>> Hlevels;
	REGION_BD region_bd;
	int N_REGION_W;
	int N_REGION_PSI;
	int N_REGION_SPD;
	int N_REGION_TOTAL;
};


} /* namespace srcl */

#endif /* HCOST_TILE_LIBRARY_HPP */
