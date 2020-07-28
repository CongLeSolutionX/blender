/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2001-2002 by NaN Holding BV.
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): Aurel Gruber
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#include "uv_initializer.h"
#include<Eigen/SparseLU>

#include "igl/Timer.h"
#include "cotmatrix.h"
#include <iostream>

using namespace std;

double computeAngle(const Eigen::Vector3d &a,const Eigen::Vector3d &b){
	return acos(a.dot(b) / (a.norm() * b.norm()));
}

void findVertexToOppositeAnglesCorrespondence(const Eigen::MatrixXi &F,const Eigen::MatrixXd &V, Eigen::SparseMatrix<double> &vertexToFaceIndices){

	typedef Eigen::Triplet<double> T;
	std::vector<T> coefficients;

	for (int i = 0; i < F.rows(); i++){

		int vertexIndex1 = F(i,0); // retrieve vertex index that is part of face
		int vertexIndex2 = F(i,1); // retrieve vertex index that is part of face
		int vertexIndex3 = F(i,2); // retrieve vertex index that is part of face

		double angle1 = computeAngle(V.row(vertexIndex2) - V.row(vertexIndex1), V.row(vertexIndex3) - V.row(vertexIndex1));
		double angle2 = computeAngle(V.row(vertexIndex3) - V.row(vertexIndex2), V.row(vertexIndex1) - V.row(vertexIndex2));
		double angle3 = computeAngle(V.row(vertexIndex1) - V.row(vertexIndex3), V.row(vertexIndex2) - V.row(vertexIndex3));

		coefficients.push_back(T(vertexIndex1, 2*vertexIndex2, angle3));
		coefficients.push_back(T(vertexIndex1, 2*vertexIndex3 + 1, angle2));

		coefficients.push_back(T(vertexIndex2, 2*vertexIndex1 + 1, angle3));
		coefficients.push_back(T(vertexIndex2, 2*vertexIndex3, angle1));

		coefficients.push_back(T(vertexIndex3, 2*vertexIndex1, angle2));
		coefficients.push_back(T(vertexIndex3, 2*vertexIndex2 + 1, angle1));

	}

	vertexToFaceIndices.setFromTriplets(coefficients.begin(), coefficients.end());
}

void findVertexToItsAnglesCorrespondence(const Eigen::MatrixXi &F,const Eigen::MatrixXd &V, Eigen::SparseMatrix<double> &vertexToFaceIndices){

	typedef Eigen::Triplet<double> T;
	std::vector<T> coefficients;

	for (int i = 0; i < F.rows(); i++){

		int vertexIndex1 = F(i,0); // retrieve vertex index that is part of face
		int vertexIndex2 = F(i,1); // retrieve vertex index that is part of face
		int vertexIndex3 = F(i,2); // retrieve vertex index that is part of face

		double angle1 = computeAngle(V.row(vertexIndex2) - V.row(vertexIndex1), V.row(vertexIndex3) - V.row(vertexIndex1));
		double angle2 = computeAngle(V.row(vertexIndex3) - V.row(vertexIndex2), V.row(vertexIndex1) - V.row(vertexIndex2));
		double angle3 = computeAngle(V.row(vertexIndex1) - V.row(vertexIndex3), V.row(vertexIndex2) - V.row(vertexIndex3));

		coefficients.push_back(T(vertexIndex1, 2*vertexIndex2, angle1));
		coefficients.push_back(T(vertexIndex1, 2*vertexIndex3 + 1, angle1));

		coefficients.push_back(T(vertexIndex2, 2*vertexIndex1 + 1, angle2));
		coefficients.push_back(T(vertexIndex2, 2*vertexIndex3, angle2));

		coefficients.push_back(T(vertexIndex3, 2*vertexIndex1, angle3));
		coefficients.push_back(T(vertexIndex3, 2*vertexIndex2 + 1, angle3));

	}

	vertexToFaceIndices.setFromTriplets(coefficients.begin(), coefficients.end());
}

/*
	Implementation of different fixed-border parameterizations, Mean Value Coordinates, Harmonic, Tutte.
 */
void UVInitializer::convex_border_parameterization(const Eigen::MatrixXi &F,
												   const Eigen::MatrixXd &V,
												   const Eigen::MatrixXi &E,
												   const Eigen::VectorXd &EL,
												   const Eigen::VectorXi &bnd,
												   const Eigen::MatrixXd &bnd_uv,
												   Eigen::MatrixXd &UV,
												   Method method){

	int nVerts = UV.rows();
	int nEdges = E.rows();

	Eigen::SparseMatrix<double> vertexToAngles(nVerts, nVerts*2);

	switch (method){
		case HARMONIC:
			findVertexToOppositeAnglesCorrespondence(F, V, vertexToAngles);
			break;
		case MVC:
			findVertexToItsAnglesCorrespondence(F, V, vertexToAngles);
			break;
		case TUTTE:
			break;
	}

	int nUnknowns = nVerts - bnd.size();
	int nKnowns = bnd.size();

	Eigen::SparseMatrix<double> Aint(nUnknowns, nUnknowns);
	Eigen::SparseMatrix<double> Abnd(nUnknowns, nKnowns);
	Eigen::VectorXd	z(nKnowns);

	std::vector<Eigen::Triplet<double>> intTripletVector;
	std::vector<Eigen::Triplet<double>> bndTripletVector;

	int rowindex;
	int columnindex;
	double edgeWeight, edgeLength;
	Eigen::RowVector2i edge;

	int firstVertex, secondVertex;

	for (int e = 0; e < nEdges; e++){
		edge = E.row(e);
		edgeLength = EL(e);
		firstVertex = edge(0);
		secondVertex = edge(1);

		if (firstVertex >= nKnowns){//into Aint

			rowindex = firstVertex - nKnowns;

			double angle1 = vertexToAngles.coeff(firstVertex, 2*secondVertex);
			double angle2 = vertexToAngles.coeff(firstVertex, 2*secondVertex+1);

			switch (method){
				case HARMONIC:
					edgeWeight = 1/tan(angle1) + 1/tan(angle2);
					break;
				case MVC:
					edgeWeight = tan(angle1/2) + tan(angle2/2);
					edgeWeight /= edgeLength;
					break;
				case TUTTE:
					edgeWeight = 1;
					break;
			}

			intTripletVector.push_back(Eigen::Triplet<double>(rowindex, rowindex, edgeWeight));

			if (secondVertex >= nKnowns){ // also an unknown point in the interior
				columnindex = secondVertex - nKnowns;

				intTripletVector.push_back(Eigen::Triplet<double>(rowindex, columnindex, -edgeWeight));

			} else { // known point on the border
				columnindex = secondVertex;
				bndTripletVector.push_back(Eigen::Triplet<double>(rowindex, columnindex, edgeWeight));
			}

		}
	}


	Aint.setFromTriplets(intTripletVector.begin(), intTripletVector.end());
	Aint.makeCompressed();

	Abnd.setFromTriplets(bndTripletVector.begin(), bndTripletVector.end());
	Abnd.makeCompressed();

	for (int i = 0; i < nUnknowns; i++){
		double factor = Aint.coeff(i, i);
		Aint.row(i) /= factor;
		Abnd.row(i) /= factor;
	}

	Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
	solver.compute(Aint);

	for (int i = 0; i<2; i++){// solve for u and v coordinates

		for (int zindex = 0; zindex < nKnowns; zindex++){
			z(zindex) = bnd_uv(bnd(zindex), i);
		}

		Eigen::VectorXd b = Abnd*z;

		Eigen::VectorXd uvs;
		uvs = solver.solve(b);

		Eigen::VectorXd boundary = bnd_uv.col(i);
		Eigen::VectorXd interior = uvs;

		UV.col(i) << boundary, interior;
	}
}

void UVInitializer::mvc(const Eigen::MatrixXi &F,
						const Eigen::MatrixXd &V,
						const Eigen::MatrixXi &E,
						const Eigen::VectorXd &EL,
						const Eigen::VectorXi &bnd,
						const Eigen::MatrixXd &bnd_uv,
						Eigen::MatrixXd &UV){

	UVInitializer::convex_border_parameterization(F, V, E, EL, bnd, bnd_uv, UV, Method::MVC);
}

void UVInitializer::harmonic(const Eigen::MatrixXi &F,
						const Eigen::MatrixXd &V,
						const Eigen::MatrixXi &E,
						const Eigen::VectorXd &EL,
						const Eigen::VectorXi &bnd,
						const Eigen::MatrixXd &bnd_uv,
						Eigen::MatrixXd &UV){

	UVInitializer::convex_border_parameterization(F, V, E, EL, bnd, bnd_uv, UV, Method::HARMONIC);
}

void UVInitializer::tutte(const Eigen::MatrixXi &F,
						const Eigen::MatrixXd &V,
						const Eigen::MatrixXi &E,
						const Eigen::VectorXd &EL,
						const Eigen::VectorXi &bnd,
						const Eigen::MatrixXd &bnd_uv,
						Eigen::MatrixXd &UV){

	UVInitializer::convex_border_parameterization(F, V, E, EL, bnd, bnd_uv, UV, Method::TUTTE);
}

void UVInitializer::mapVerticesToConvexBorder(Eigen::MatrixXd &vertex_positions) {
	double pi = atan(1)*4;
	int n_boundary_vertices = vertex_positions.rows();
	double x, y;
	double angle = 2*pi / n_boundary_vertices;

	for (int i = 0; i < n_boundary_vertices; i++) {
		x = cos(angle*i);
		y = sin(angle*i);
		vertex_positions(i,0) = (x*0.5) + 0.5;
		vertex_positions(i,1) = (y*0.5) + 0.5;
	}
}

void get_flips(const Eigen::MatrixXd& V,
							  const Eigen::MatrixXi& F,
							  const Eigen::MatrixXd& uv,
							  std::vector<int>& flip_idx) {
	flip_idx.resize(0);
	for (int i = 0; i < F.rows(); i++) {

		Eigen::Vector2d v1_n = uv.row(F(i,0)); Eigen::Vector2d v2_n = uv.row(F(i,1)); Eigen::Vector2d v3_n = uv.row(F(i,2));

		Eigen::MatrixXd T2_Homo(3,3);
		T2_Homo.col(0) << v1_n(0),v1_n(1),1;
		T2_Homo.col(1) << v2_n(0),v2_n(1),1;
		T2_Homo.col(2) << v3_n(0),v3_n(1),1;
		double det = T2_Homo.determinant();
		assert (det == det);
		if (det < 0) {
			//cout << "flip at face #" << i << " det = " << T2_Homo.determinant() << endl;
			flip_idx.push_back(i);
		}
	}
}

int UVInitializer::count_flips(const Eigen::MatrixXd& V,
				const Eigen::MatrixXi& F,
				const Eigen::MatrixXd& uv) {

	std::vector<int> flip_idx;
	get_flips(V,F,uv,flip_idx);

	return flip_idx.size();
}



