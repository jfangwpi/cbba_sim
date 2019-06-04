/*
 * defn_tile.cpp
 *
 *  Created on: Mar 14, 2016
 *      Author: bscooper
 *
 *  Functions for Tile class
 */

// Eigen header
//#include <eigen3/Eigen/Core>

#include "cbta/hcost_tile_library.hpp"
#include <sstream>

using namespace Eigen;
using namespace librav;

/* ------- Tile constructors and destructors -------- */
Tile::Tile(int H, Matrix<int,Dynamic,4> tile_vertices){
	FACE_REF <<   1,  -1,   1,   0,   0,
				 -2,   2,   1,   2,   0,
				  2,  -2,   1,   1,   0,
				 -1,   1,   1,   4,   0,
				  1,  -2,   2,   0,   0,
				 -2,  -1,   2,   2,   0,
				  2,   1,   2,   1,   0,
				 -1,  -2,   2,   4,   0,
				  1,   2,   2,   3,   0,
				  2,  -1,   2,   4,   2,
				 -2,   1,   2,   4,   1,
				 -1,   2,   2,   4,   3;
	VERTICES_PERMUTATION << 1, 2, 3, 4,
							4, 1, 2, 3,
							2, 3, 4, 1,
							4, 3, 2, 1,
							2, 1, 4, 3;
	INVERSE_XFORM << 0,2,1,3,4;
	set_tile_data(H,tile_vertices);
	//std::shared_ptr<TileBlock> tile_block;
}

Tile::Tile(std::string traversal_type_str,
		   std::string traversal_faces_str,
		   std::string cell_xform_str,
		   std::string channel_data_str,
		   std::string cell_edge_str,
		   std::string cell_vertices_str,
		   std::string connectivity_str){
		//    std::string FACE_REF_str,
		//    std::string VERTICES_PERMUTATION_str,
		//    std::string INVERSE_XFORM_str){

	setMatricesFromJSON(traversal_type_str,
						traversal_faces_str,
						cell_xform_str,
						channel_data_str,
						cell_edge_str,
						cell_vertices_str);
						// FACE_REF_str,
						// VERTICES_PERMUTATION_str,
						// INVERSE_XFORM_str);
						
	setConnectivityFromJSON(connectivity_str);

	FACE_REF <<   1,  -1,   1,   0,   0,
				 -2,   2,   1,   2,   0,
				  2,  -2,   1,   1,   0,
				 -1,   1,   1,   4,   0,
				  1,  -2,   2,   0,   0,
				 -2,  -1,   2,   2,   0,
				  2,   1,   2,   1,   0,
				 -1,  -2,   2,   4,   0,
				  1,   2,   2,   3,   0,
				  2,  -1,   2,   4,   2,
				 -2,   1,   2,   4,   1,
				 -1,   2,   2,   4,   3;
	VERTICES_PERMUTATION << 1, 2, 3, 4,
							4, 1, 2, 3,
							2, 3, 4, 1,
							4, 3, 2, 1,
							2, 1, 4, 3;
	INVERSE_XFORM << 0,2,1,3,4;
}


Tile::~Tile(){

}

void Tile::set_tile_data(int H, Matrix<int,Dynamic,4> tile_vertices){
	// ---- Location of cells
	Matrix<double,4,4> I4 = MatrixXd::Identity(4,4);
	traversal_type = MatrixXi::Zero(H,1);  // Opposite (1), adjacent (2)
	cell_xform 	   = MatrixXi::Zero(H,2); // Transformations to bring to standard

	MatrixXi chan_mid(H,4);
	MatrixXi middiag = tile_vertices.block(1,2,H,1).asDiagonal();
	chan_mid.leftCols(2) << middiag*MatrixXi::Ones(H,2); //rect dims (dx,dy)
	chan_mid.rightCols(2) << tile_vertices.block(1,0,H,2);                                //rect center coords
//	std::cout << "chan_mid" << std::endl << chan_mid << std::endl;

	MatrixXd chan_full(H+2,4);
	MatrixXi fulldiagint = tile_vertices.col(2).asDiagonal();
	MatrixXd fulldiag = fulldiagint.cast<double>();
	chan_full.leftCols(2) = fulldiag*MatrixXd::Ones(H+2,2); //Full channel
	chan_full.rightCols(2) = tile_vertices.leftCols(2).cast<double>();						// needed only to define entry/exit segments
//	std::cout << "chan_full" << std::endl << chan_full << std::endl;
	channel_data = chan_mid;
	// Entry segments
	MatrixXi u1(H,1);
	u1 = MatrixXi::Zero(H+1,1);
	MatrixXi  exit_seg(H+1,1);
	exit_seg  = MatrixXi::Zero(H+1,1);
	cell_edge = MatrixXd::Zero(H+1,4);

	for(int n=1; n < H+2; n++){
		double del_X = chan_full(n,2) - chan_full(n-1,2);
		double tol = 1e-6;

		if (abs(abs(del_X) - chan_full.block(n-1,0,2,1).sum()/2.0) < tol){
			//std::cout << "X transition" << std::endl;
			if (chan_full(n,2) > chan_full(n-1,2)){					// X transition
				exit_seg(n-1,0) = -1;								// Right
			}else{
				exit_seg(n-1,0) = 1;								// Left
			}
			u1(n-1,0) = 0;
			double x_lim = chan_full(n-1,2) - exit_seg(n-1,0)*chan_full(n-1,0)/2.0;
			Vector2d y_lim, y_lim_min, y_lim_max;
			y_lim_min << chan_full(n,3) + chan_full(n,1)/2, chan_full(n-1,3) + chan_full(n-1,1)/2;
			y_lim_max << chan_full(n,3) - chan_full(n,1)/2, chan_full(n-1,3) - chan_full(n-1,1)/2;
			y_lim << y_lim_min.minCoeff(), y_lim_max.maxCoeff();
			cell_edge.row(n-1) << x_lim, y_lim(0), x_lim, y_lim(1);
		}else{
			//std::cout << "Y transition" << std::endl;
			if (chan_full(n,3) > chan_full(n-1,3)){					// Y transition
				exit_seg(n-1,0) = 2;								// Up
			}else{
				exit_seg(n-1,0) = -2;								// Down
			}
			u1(n-1,0) = 1;
			double y_lim = chan_full(n-1,3) + exit_seg(n-1,0)/2.0*chan_full(n-1,1)/2.0;
			Vector2d x_lim, x_lim_min, x_lim_max;
			x_lim_min << chan_full(n,2) + chan_full(n,0)/2, chan_full(n-1,2) + chan_full(n-1,0)/2;
			x_lim_max << chan_full(n,2) - chan_full(n,0)/2, chan_full(n-1,2) - chan_full(n-1,0)/2;
			x_lim << x_lim_min.minCoeff(), x_lim_max.maxCoeff();

			cell_edge.row(n-1) << x_lim(0), y_lim, x_lim(1), y_lim;
		}

	}
	VectorXi face_from = -exit_seg.topRows(H);
	VectorXi face_to   = exit_seg.bottomRows(H);\
	traversal_faces.resize(2*H,1);
	traversal_faces << face_from, face_to;
//	std::cout << face_from << std::endl;
//	std::cout << face_to << std::endl;

	// Type of transition
	cell_vertices = MatrixXd::Zero(4,2*H);   // Vertices of each rectangle, side by side
//	std::cout << "cell_vertices" << std::endl;
//	std::cout << cell_vertices << std::endl;
	for (int n=0; n < H; n++){
		Matrix<double,4,2> this_cell_vertices;
		MatrixXd chan_mid_d(H,4);
		chan_mid_d = chan_mid.cast<double>();
		this_cell_vertices.row(0) << chan_mid_d(n,2) - chan_mid_d(n,0)/2.0, chan_mid_d(n,3) + chan_mid_d(n,1)/2.0; // Vertices of the // rectangle in CW
		this_cell_vertices.row(1) << chan_mid_d(n,2) + chan_mid_d(n,0)/2.0, chan_mid_d(n,3) + chan_mid_d(n,1)/2.0; // order starting // with top left
		this_cell_vertices.row(2) << chan_mid_d(n,2) + chan_mid_d(n,0)/2.0, chan_mid_d(n,3) - chan_mid_d(n,1)/2.0;
		this_cell_vertices.row(3) << chan_mid_d(n,2) - chan_mid_d(n,0)/2.0, chan_mid_d(n,3) - chan_mid_d(n,1)/2.0;
//		std::cout << "this_cell_vertices" << std::endl;
//		std::cout << this_cell_vertices << std::endl;
		Matrix<int,1,2> face;
		int idx1;
		for (int mm = 0; mm < 12; mm++){
			face(0,0) = face_from(n);
			face(0,1) = face_to(n);
			if(face.isApprox(FACE_REF.row(mm).leftCols(2))){
				idx1 = mm;
				break;
			}
		}
//		std::cout << "idx1 = " << idx1 << std::endl;
		cell_xform.row(n) = FACE_REF.row(idx1).rightCols(2); // cols 4,5, Existing transformation on current rectangle
//		std::cout << "cell_xform" << std::endl;
//		std::cout << cell_xform << std::endl;
		traversal_type(n,0) = (u1(n,0)!=u1(n+1,0)) + 1;
//		std::cout << "traversal_type" << std::endl;
//		std::cout << traversal_type << std::endl;

		int xform1 = INVERSE_XFORM(FACE_REF(idx1,4));
		int xform2 = INVERSE_XFORM(FACE_REF(idx1,3));
//		std::cout << "xform1 = " << xform1 << std::endl;
//		std::cout << "xform2 = " << xform2 << std::endl;
		RowVectorXi p1 = VERTICES_PERMUTATION.row(xform1);
		RowVectorXi p2 = VERTICES_PERMUTATION.row(xform2);
//		std::cout << "p1 = " << p1 << std::endl;
//		std::cout << "p2 = " << p2 << std::endl;

		Matrix4d I4p1;
		int p10 = p1(0); int p11 = p1(1); int p12 = p1(2); int p13 = p1(3);
		I4p1.row(0) = I4.row(p10-1);
		I4p1.row(1) = I4.row(p11-1);
		I4p1.row(2) = I4.row(p12-1);
		I4p1.row(3) = I4.row(p13-1);
		//std::cout << "I4p1" << std::endl << I4p1 << std::endl;
		Matrix4d I4p2;
		int p20 = p2(0); int p21 = p2(1); int p22 = p2(2); int p23 = p2(3);
		I4p2.row(0) = I4.row(p20-1);
		I4p2.row(1) = I4.row(p21-1);
		I4p2.row(2) = I4.row(p22-1);
		I4p2.row(3) = I4.row(p23-1);
		//std::cout << "I4p2" << std::endl << I4p2 << std::endl;
		MatrixXd this_cell_aug_vertices;
		this_cell_aug_vertices = I4p2*I4p1*this_cell_vertices;
		//std::cout << this_cell_aug_vertices << std::endl;
		cell_vertices.block(0,2*n,4,2) = this_cell_aug_vertices;
	}

}

void Tile::addTileBlock(REGION_BD &REGION_BD, std::shared_ptr<TileBlock> this_tile_block, int Hin){
	//tile_block = std::make_shared<TileBlock>(REGION_BD, this_tile, Hin);
	tile_block = this_tile_block;
}

void Tile::setMatricesFromJSON(std::string traversal_type_str,
							   std::string traversal_faces_str,
							   std::string cell_xform_str,
							   std::string channel_data_str,
							   std::string cell_edge_str,
							   std::string cell_vertices_str){
							//    std::string FACE_REF_str,
							//    std::string VERTICES_PERMUTATION_str,
							//    std::string INVERSE_XFORM_str){

	/******* cell_xform_str *******/
	char temp_char[cell_xform_str.size() +1];
	strcpy(temp_char, cell_xform_str.c_str());
	char *token;
	
	std::vector<char *> burner_array;
	token = strtok (temp_char,"[");

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
	// cell_xform = MatrixXi::Zero(counter,n);
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
			this->cell_xform(i,k) = ind_num_array[k];
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
	// channel_data = MatrixXi::Zero(counter,n);
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
			this->channel_data(i,k) = ind_num_array[k];
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
	// traversal_faces = MatrixXi::Zero(counter,n);
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
			this->traversal_faces(i,k) = ind_num_array[k];
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
	// traversal_type = MatrixXi::Zero(counter,n);
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
			this->traversal_type(i,k) = ind_num_array[k];
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

	// cell_edge = MatrixXd::Zero(counter,n);
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
			this->cell_edge(i,k) = ind_num_array[k];
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
	//cell_vertices = MatrixXd::Zero(counter,n);
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
			this->cell_vertices(i,k) = ind_num_array[k];
			// std::cout << cell_vertices(i,k) << std::endl;
		}
	}
}

void Tile::setConnectivityFromJSON(std::string connectivity_str){

	// std::shared_ptr<Eigen::SparseMatrix<int,Eigen::RowMajor>> connectivity = std::make_shared<Eigen::SparseMatrix<int,Eigen::RowMajor>>(2500,2500);

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
	
	this->connectivity = std::make_shared<Eigen::SparseMatrix<int,Eigen::RowMajor>>(2500,2500);
	this->connectivity->setFromTriplets(tripletList.begin(), tripletList.end());
	// std::cout << connectivity->nonZeros() << std::endl;

}