#pragma once
#include <vector>
#include <glm\glm.hpp>
#include <GL\glew.h>

// Parametros del thresholding
#define TH_DIV_S  12
#define TH_T  0.15f

// Constantes usadas para los anillos
#define R_PAR_MIN_ASPECT_RATIO      0.5     // Factor aspect ratio minimo del anillo padre
#define R_CHD_MIN_ASPECT_RATIO      0.4
#define R_CUR_MIN_ASPECT_RATIO      0.55
#define R_PAR_MIN_RECTAN    0.7     // Factor de rectangularidad
#define R_CHD_MIN_RECTAN    0.4
#define R_CUR_MIN_RECTAN    0.75
#define R_CUR_MIN_AREA      0.1
#define NUM_ANILLOS 30
//#define NUM_ROW_ANILL 5
//#define NUM_COL_ANILL 6

#define NUM_ROW_ANILL 3
#define NUM_COL_ANILL 4

// Parametros de contornos
#define CON_MIN_SIZE_CONTOUR    6
#define CON_MYEPS 1e-8

// Parametros estadistico
#define PR_ERRORF          0.0001
#define PR_AUTO_INTERVAL   -1

//operaciones
#define OP_INF (1<<30)

//colores para el etiquetado
#define MY_COLOR_GREEN CV_RGB(20,150,20)
#define MY_COLOR_BLUE CV_RGB(0,0,205)
#define MY_COLOR_YELLOW CV_RGB(255,255,0)
#define MY_COLOR_RED CV_RGB(255,0,0)
#define MY_COLOR_WHITE CV_RGB(255,255,255)
#define MY_COLOR_ORANGE CV_RGB(255,69,0)

// Variables Opengl
extern glm::mat4 Projection;
extern glm::mat4 View;
extern glm::mat4 Model;

struct ShaderInfo {
	GLuint shaderID;
	std::vector<GLuint> uniformInfo;
};
