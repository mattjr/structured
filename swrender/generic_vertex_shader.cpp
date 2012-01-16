/*
    Fusion2X - OpenGL ES-CL 1.0 Implementation
    Copyright (C) 2008 Markus Trenkwalder

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

    Markus Trenkwalder
    trenki2@gmx.net
*/

#include "generic_vertex_shader.h"
vcg::Matrix44d GenericVertexShader::modelview_projection_matrix_d;

mat4x GenericVertexShader::modelview_projection_matrix;
void printmatrix2(mat4x mat){

for(int i=0; i<4; i++){
    for(int j=0; j<4; j++){
        std::cout << std::fixed << std::setprecision(4) << std::setw(7)<<fix2float<16>(mat.elem[i][j].intValue) << " ";
    }
    std::cout <<std::endl;
}
}


void printmatrix3(vcg::Matrix44d mat){

for(int i=0; i<4; i++){
    for(int j=0; j<4; j++){
        std::cout << std::fixed << std::setprecision(4) << std::setw(7)<<mat.ElementAt(i,j) << " ";
    }
    std::cout <<std::endl;
}
}
