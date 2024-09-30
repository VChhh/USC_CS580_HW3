/* CS580 Homework 3 */

#include	"stdafx.h"
#include	"stdio.h"
#include	"math.h"
#include	"Gz.h"
#include	"rend.h"

#include "Vector3.h" // self-defined vector3 class
#include "Vector4.h" // self-defined vector4 class

#define PI (float) 3.14159265358979323846

// matrix multiplication
void matrix_multiply(GzMatrix mat1, GzMatrix mat2, GzMatrix result) {
	for (size_t i = 0; i < 4; ++i) {
		for (size_t j = 0; j < 4; ++j) {
			float sum = 0;
			for (size_t k = 0; k < 4; ++k) {
				sum += mat1[i][k] * mat2[k][j];
			}
			result[i][j] = sum;
		}
	}
}

// transform the vector by the matrix
template<typename T>
Vector4<T> transform_vector4D(Vector4<T> vector, GzMatrix mat) {
	Vector4<T> result;
	for (size_t i = 0; i < 4; ++i) {
		T sum = 0;
		for (size_t j = 0; j < 4; ++j) {
			sum += mat[i][j] * vector[j];
		}
		result[i] = sum;
	}
	return result;
}



int GzRender::GzRotXMat(float degree, GzMatrix mat)
{
/* HW 3.1
// Create rotate matrix : rotate along x axis
// Pass back the matrix using mat value
*/
	float radian = degree * PI / 180.0f;
	// initialize the matrix
	for (size_t i = 0; i < 4; ++i) {
		for (size_t j = 0; j < 4; ++j) {
			mat[i][j] = 0;
		}
	}
	mat[0][0] = 1.0f;
	mat[1][1] = cos(radian);
	mat[1][2] = -sin(radian);
	mat[2][1] = sin(radian);
	mat[2][2] = cos(radian);
	mat[3][3] = 1.0f;
	return GZ_SUCCESS;
}

int GzRender::GzRotYMat(float degree, GzMatrix mat)
{
/* HW 3.2
// Create rotate matrix : rotate along y axis
// Pass back the matrix using mat value
*/
	float radian = degree * PI / 180.0f;
	// initialize the matrix
	for (size_t i = 0; i < 4; ++i) {
		for (size_t j = 0; j < 4; ++j) {
			mat[i][j] = 0;
		}
	}
	mat[0][0] = cos(radian);
	mat[0][2] = sin(radian);
	mat[1][1] = 1.0f;
	mat[2][0] = -sin(radian);
	mat[2][2] = cos(radian);
	mat[3][3] = 1.0f;
	return GZ_SUCCESS;
}

int GzRender::GzRotZMat(float degree, GzMatrix mat)
{
/* HW 3.3
// Create rotate matrix : rotate along z axis
// Pass back the matrix using mat value
*/
	float radian = degree * PI / 180.0f;
	// initialize the matrix
	for (size_t i = 0; i < 4; ++i) {
		for (size_t j = 0; j < 4; ++j) {
			mat[i][j] = 0;
		}
	}
	mat[0][0] = cos(radian);
	mat[0][1] = -sin(radian);
	mat[1][0] = sin(radian);
	mat[1][1] = cos(radian);
	mat[2][2] = 1.0f;
	mat[3][3] = 1.0f;
	return GZ_SUCCESS;
}

int GzRender::GzTrxMat(GzCoord translate, GzMatrix mat)
{
/* HW 3.4
// Create translation matrix
// Pass back the matrix using mat value
*/
	// initialize the matrix
	for (size_t i = 0; i < 4; ++i) {
		for (size_t j = 0; j < 4; ++j) {
			mat[i][j] = 0;
		}
	}
	mat[0][0] = 1.0f;
	mat[1][1] = 1.0f;
	mat[2][2] = 1.0f;
	mat[3][3] = 1.0f;
	mat[0][3] = translate[X];
	mat[1][3] = translate[Y];
	mat[2][3] = translate[Z];
	return GZ_SUCCESS;
}


int GzRender::GzScaleMat(GzCoord scale, GzMatrix mat)
{
/* HW 3.5
// Create scaling matrix
// Pass back the matrix using mat value
*/
	// initialize the matrix
	for (size_t i = 0; i < 4; ++i) {
		for (size_t j = 0; j < 4; ++j) {
			mat[i][j] = 0;
		}
	}
	mat[0][0] = scale[X];
	mat[1][1] = scale[Y];
	mat[2][2] = scale[Z];
	mat[3][3] = 1.0f;
	return GZ_SUCCESS;
}


GzRender::GzRender(int xRes, int yRes)
{
/* HW1.1 create a framebuffer for MS Windows display:
 -- set display resolution
 -- allocate memory for framebuffer : 3 bytes(b, g, r) x width x height
 -- allocate memory for pixel buffer
 */
	xres = xRes <= 0 ? 0 : (xRes > MAXXRES ? MAXXRES : xRes);
	yres = yRes <= 0 ? 0 : (yRes > MAXYRES ? MAXYRES : yRes);
	framebuffer = (char*) malloc (3 * sizeof(char) * xRes * yRes); // provided
	// framebuffer = new char[3 * xres * yres];
	pixelbuffer = new GzPixel[xres * yres];

/* HW 3.6
- setup Xsp and anything only done once 
- init default camera 
*/ 
	matlevel = -1;

	// assign 0 to all Xsp elements
	for (size_t i = 0; i < 4; ++i) {
		for (size_t j = 0; j < 4; ++j) {
			Xsp[i][j] = 0;
		}
	}

	// set up Xsp
	Xsp[0][0] = xres / 2.0f;
	Xsp[1][1] = -yres / 2.0f;
	Xsp[2][2] = MAXINT;
	Xsp[0][3] = xres / 2.0f;
	Xsp[1][3] = yres / 2.0f;
	Xsp[3][3] = 1.0f;

	// set up default camera
	m_camera.position[X] = DEFAULT_IM_X;
	m_camera.position[Y] = DEFAULT_IM_Y;
	m_camera.position[Z] = DEFAULT_IM_Z;
	m_camera.lookat[X] = 0.0f;
	m_camera.lookat[Y] = 0.0f;
	m_camera.lookat[Z] = 0.0f;
	m_camera.worldup[X] = 0.0f;
	m_camera.worldup[Y] = 1.0f;
	m_camera.worldup[Z] = 0.0f;
	m_camera.FOV = DEFAULT_FOV;
}

GzRender::~GzRender()
{
/* HW1.2 clean up, free buffer memory */
	delete[] framebuffer;
	delete[] pixelbuffer;
}

int GzRender::GzDefault()
{
/* HW1.3 set pixel buffer to some default values - start a new frame */
	for (size_t i = 0; i < xres * yres; ++i) {
		framebuffer[3 * i] = 0;
		framebuffer[3 * i + 1] = 0;
		framebuffer[3 * i + 2] = 0;
		pixelbuffer[i] = { 2055, 1798, 1541, 0, INT_MAX };
	}
	return GZ_SUCCESS;
}

int GzRender::GzBeginRender()
{
/* HW 3.7 
- setup for start of each frame - init frame buffer color,alpha,z
- compute Xiw and projection xform Xpi from camera definition 
- init Ximage - put Xsp at base of stack, push on Xpi and Xiw 
- now stack contains Xsw and app can push model Xforms when needed 
*/ 
	// initialize Xpi and Xiw
	GzMatrix Xpi;
	GzMatrix Xiw;
	for (size_t i = 0; i < 4; ++i) {
		for (size_t j = 0; j < 4; ++j) {
			Xpi[i][j] = 0;
			Xiw[i][j] = 0;
		}
	}

	// construct Xpi
	// let dd = 1/d = tan(fov/2)
	float dd = tan(m_camera.FOV * PI / 180.0f / 2.0f);
	Xpi[0][0] = 1.0f;
	Xpi[1][1] = 1.0f;
	Xpi[2][2] = dd;
	Xpi[3][2] = dd;
	Xpi[3][3] = 1.0f;

	// construct Xiw
	// get the camera information in vector3f form
	Vector3f vector_lookat = Vector3f(m_camera.lookat[X], m_camera.lookat[Y], m_camera.lookat[Z]);
	Vector3f vector_position = Vector3f(m_camera.position[X], m_camera.position[Y], m_camera.position[Z]);
	Vector3f vector_worldup = Vector3f(m_camera.worldup[X], m_camera.worldup[Y], m_camera.worldup[Z]);

	// Z axis
	Vector3f Z_axis = (vector_lookat - vector_position).normalize();

	// up
	Vector3f up_modified = vector_worldup - Z_axis * vector_worldup.dot(Z_axis);

	// Y axis
	Vector3f Y_axis = up_modified.normalize();

	// X axis
	Vector3f X_axis = Y_axis.cross(Z_axis);

	Xiw[0][0] = X_axis[0];
	Xiw[0][1] = X_axis[1];
	Xiw[0][2] = X_axis[2];
	Xiw[0][3] = -X_axis.dot(vector_position);
	Xiw[1][0] = Y_axis[0];
	Xiw[1][1] = Y_axis[1];
	Xiw[1][2] = Y_axis[2];
	Xiw[1][3] = -Y_axis.dot(vector_position);
	Xiw[2][0] = Z_axis[0];
	Xiw[2][1] = Z_axis[1];
	Xiw[2][2] = Z_axis[2];
	Xiw[2][3] = -Z_axis.dot(vector_position);
	Xiw[3][3] = 1.0f;

	//put Xpi and Xiw to the camera
	for (size_t i = 0; i < 4; ++i) {
		for (size_t j = 0; j < 4; ++j) {
			m_camera.Xiw[i][j] = Xiw[i][j];
			m_camera.Xpi[i][j] = Xpi[i][j];
		}
	}

	// push into the stack
	GzPushMatrix(Xsp);
	GzPushMatrix(Xpi);
	GzPushMatrix(Xiw);

	return GZ_SUCCESS;
}

int GzRender::GzPutCamera(GzCamera camera)
{
/* HW 3.8 
/*- overwrite renderer camera structure with new camera definition
*/
	for (size_t i = 0; i < 4; ++i) {
		for (size_t j = 0; j < 4; ++j) {
			m_camera.Xiw[i][j] = camera.Xiw[i][j];
			m_camera.Xpi[i][j] = camera.Xpi[i][j];
		}
	}
	for (size_t i = 0; i < 3; ++i) {
		m_camera.position[i] = camera.position[i];
		m_camera.lookat[i] = camera.lookat[i];
		m_camera.worldup[i] = camera.worldup[i];
	}
	m_camera.FOV = camera.FOV;
	return GZ_SUCCESS;	
}

int GzRender::GzPushMatrix(GzMatrix	matrix)
{
/* HW 3.9 
- push a matrix onto the Ximage stack
- check for stack overflow
*/
	// check for stack overflow
	if (matlevel >= MATLEVELS) return GZ_FAILURE;

	// push the matrix
	if (matlevel == -1) {
		// put the matrix at the base of the stack
		for (size_t i = 0; i < 4; ++i) {
			for (size_t j = 0; j < 4; ++j) {
				Ximage[0][i][j] = matrix[i][j];
			}
		}
	}
	else {
		// multiply the matrix with the top of the stack
		GzMatrix result;
		matrix_multiply(Ximage[matlevel], matrix, result);
		for (size_t i = 0; i < 4; ++i) {
			for (size_t j = 0; j < 4; ++j) {
				Ximage[matlevel + 1][i][j] = result[i][j];
			}
		}
	}
	// increment the matlevel
	++matlevel;
	return GZ_SUCCESS;
}

int GzRender::GzPopMatrix()
{
/* HW 3.10
- pop a matrix off the Ximage stack
- check for stack underflow
*/
	// check for stack underflow
	if (matlevel < 0) return GZ_FAILURE;
	--matlevel;
	return GZ_SUCCESS;
}

int GzRender::GzPut(int i, int j, GzIntensity r, GzIntensity g, GzIntensity b, GzIntensity a, GzDepth z)
{
/* HW1.4 write pixel values into the buffer */
	if (pixelbuffer == NULL) return GZ_FAILURE;
	if (i < 0 || i >= xres || j < 0 || j >= yres) return GZ_FAILURE;

	if (z < pixelbuffer[ARRAY(i, j)].z) {
		// clamp the value to its range
		GzIntensity red = clamp(r, GzIntensity(0), GzIntensity(4095));
		GzIntensity green = clamp(g, GzIntensity(0), GzIntensity(4095));
		GzIntensity blue = clamp(b, GzIntensity(0), GzIntensity(4095));

		pixelbuffer[ARRAY(i, j)] = GzPixel{ red, green, blue, a, z };
	}
	return GZ_SUCCESS;
}


int GzRender::GzGet(int i, int j, GzIntensity *r, GzIntensity *g, GzIntensity *b, GzIntensity *a, GzDepth *z)
{
/* HW1.5 retrieve a pixel information from the pixel buffer */
	if (pixelbuffer == NULL) return GZ_FAILURE;
	if (i < 0 || i >= xres || j < 0 || j >= yres) return GZ_FAILURE;

	*r = pixelbuffer[ARRAY(i, j)].red;
	*g = pixelbuffer[ARRAY(i, j)].green;
	*b = pixelbuffer[ARRAY(i, j)].blue;
	*a = pixelbuffer[ARRAY(i, j)].alpha;
	*z = pixelbuffer[ARRAY(i, j)].z;
	return GZ_SUCCESS;
}


int GzRender::GzFlushDisplay2File(FILE* outfile)
{
/* HW1.6 write image to ppm file -- "P6 %d %d 255\r" */
	if (pixelbuffer == NULL) return GZ_FAILURE;

	fprintf(outfile, "P6 %d %d 255\r", xres, yres);

	for (size_t i = 0; i < xres * yres; ++i) {
		// convert to 8 bits
		char pixel_red = (pixelbuffer[i].red >> 4) & 0xff;
		char pixel_green = (pixelbuffer[i].green >> 4) & 0xff;
		char pixel_blue = (pixelbuffer[i].blue >> 4) & 0xff;

		// write to outfile
		fwrite(&pixel_red, 1, 1, outfile);
		fwrite(&pixel_green, 1, 1, outfile);
		fwrite(&pixel_blue, 1, 1, outfile);
	}
	return GZ_SUCCESS;
}

int GzRender::GzFlushDisplay2FrameBuffer()
{
/* HW1.7 write pixels to framebuffer: 
	- put the pixels into the frame buffer
	- CAUTION: when storing the pixels into the frame buffer, the order is blue, green, and red 
	- NOT red, green, and blue !!!
*/
	if (pixelbuffer == NULL || framebuffer == NULL) return GZ_FAILURE;

	for (size_t i = 0; i < xres * yres; ++i) {
		framebuffer[3 * i] = pixelbuffer[i].blue >> 4;
		framebuffer[3 * i + 1] = pixelbuffer[i].green >> 4;
		framebuffer[3 * i + 2] = pixelbuffer[i].red >> 4;
	}
	return GZ_SUCCESS;
}


/***********************************************/
/* HW2 methods: implement from here */

int GzRender::GzPutAttribute(int numAttributes, GzToken	*nameList, GzPointer *valueList) 
{
/* HW 2.1
-- Set renderer attribute states (e.g.: GZ_RGB_COLOR default color)
-- In later homeworks set shaders, interpolaters, texture maps, and lights
*/
	if (numAttributes <= 0 || nameList == NULL || valueList == NULL) return GZ_FAILURE;
	for (size_t i = 0; i < numAttributes; ++i) {
		if (nameList[i] == GZ_RGB_COLOR) {
			Vector3f color = *(Vector3f*)valueList[i];
			color.clamp(0.0f, 1.0f);

			// assign to flatcolor
			flatcolor[0] = color[0];
			flatcolor[1] = color[1];
			flatcolor[2] = color[2];
		}
	}
	return GZ_SUCCESS;
}

int GzRender::GzPutTriangle(int numParts, GzToken *nameList, GzPointer *valueList)
/* numParts - how many names and values */
{
/* HW 2.2
-- Pass in a triangle description with tokens and values corresponding to
      GZ_NULL_TOKEN:		do nothing - no values
      GZ_POSITION:		3 vert positions in model space
-- Invoke the rastrizer/scanline framework
-- Return error code
*/
	if (numParts <= 0 || nameList == NULL || valueList == NULL) return GZ_FAILURE;
	for (size_t i = 0; i < numParts; ++i) {
		if (nameList[i] == GZ_POSITION) {
			Vector3f* model_vertices = (Vector3f*)valueList[i];

			Vector4f vertices_4D[3];
			for (size_t j = 0; j < 3; ++j) {
				vertices_4D[j] = Vector4f(model_vertices[j].x, model_vertices[j].y, model_vertices[j].z, 1.0f);
				vertices_4D[j] = transform_vector4D(vertices_4D[j], Ximage[matlevel]);
			}

			if (vertices_4D[0].z < 0 || vertices_4D[1].z < 0 || vertices_4D[2].z < 0) return GZ_FAILURE;

			Vector3f vertices[3];
			for (size_t i = 0; i < 3; ++i) {
				vertices[i] = vertices_4D[i].toVector3Homogeneous();
			}

			// sort by y
			if (vertices[0].y > vertices[1].y) vertices[0].swap(vertices[1]);
			if (vertices[0].y > vertices[2].y) vertices[0].swap(vertices[2]);
			if (vertices[1].y > vertices[2].y) vertices[1].swap(vertices[2]);

			// check special cases
			// (int)(val + 0.5f) is to round val to the nearest integer
			// if vertices 0 and 1 are on a horizontal line
			if ((int)(vertices[0].y + 0.5f) == (int)(vertices[1].y + 0.5f)) {
				if (vertices[0].x > vertices[1].x) vertices[0].swap(vertices[1]);
			}
			// if vertices 1 and 2 are on a horizontal line
			else if ((int)(vertices[1].y + 0.5f) == (int)(vertices[2].y + 0.5f)) {
				if (vertices[2].x > vertices[1].x) vertices[2].swap(vertices[1]);
			}
			else {
				float slope_02;
				float mid_x;
				// if vertices 0 and 2 are on a vertical line
				if ((int)(vertices[0].x + 0.5f) == (int)(vertices[2].x + 0.5f)) {
					mid_x = vertices[0].x;
					slope_02 = 0.0f;
				}
				else {
					slope_02 = (vertices[2].y - vertices[0].y) / (vertices[2].x - vertices[0].x);
					mid_x = vertices[0].x + (vertices[1].y - vertices[0].y) / slope_02; // x0 + deltax
				}
				// determine the L/R edge
				if (mid_x > vertices[1].x) vertices[1].swap(vertices[2]);
			}

			// get the bounding box
			int min_x = (int)(min_by_x(vertices, 3) + 0.5f);
			int max_x = (int)(max_by_x(vertices, 3) + 0.5f);
			int min_y = (int)(min_by_y(vertices, 3) + 0.5f);
			int max_y = (int)(max_by_y(vertices, 3) + 0.5f);

			// get the edge equations
			float edge1A = vertices[1].y - vertices[0].y;
			float edge1B = vertices[0].x - vertices[1].x;
			float edge1C = -edge1B * vertices[0].y - edge1A * vertices[0].x;
			float edge2A = vertices[2].y - vertices[1].y;
			float edge2B = vertices[1].x - vertices[2].x;
			float edge2C = -edge2B * vertices[1].y - edge2A * vertices[1].x;
			float edge3A = vertices[0].y - vertices[2].y;
			float edge3B = vertices[2].x - vertices[0].x;
			float edge3C = -edge3B * vertices[2].y - edge3A * vertices[2].x;

			// get the plane equation
			Vector3f plane_normal = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
			float planeD = -(plane_normal.dot(vertices[0]));

			// rasterize
			for (int i = min_x; i <= max_x; ++i) {
				for (int j = min_y; j <= max_y; ++j) {
					// get the LEE values
					float LEE1 = edge1A * i + edge1B * j + edge1C;
					float LEE2 = edge2A * i + edge2B * j + edge2C;
					float LEE3 = edge3A * i + edge3B * j + edge3C;

					// check if in the triangle
					if ((LEE1 > 0 && LEE2 > 0 && LEE3 > 0 && plane_normal.C != 0)
						|| (LEE1 < 0 && LEE2 < 0 && LEE3 < 0 && plane_normal.C != 0)
						|| (LEE1 == 0 || LEE2 == 0 || LEE3 == 0)) {

						// calulate the z value and clip it
						int z = (int)(-(plane_normal.A * (float)i + plane_normal.B * (float)j + planeD)
							/ plane_normal.C + 0.5f);

						// put the pixel
						GzPut(i, j, ctoi(flatcolor[0]), ctoi(flatcolor[1]), ctoi(flatcolor[2]), 1, z);
					}
				}
			}
		}
	}
	return GZ_SUCCESS;
}

