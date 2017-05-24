#include <iostream>
#include<time.h>

// Include OpenCV
#include <opencv2\opencv.hpp>

// Include GLEW
#include <GL/glew.h>
#include <GL\glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// Include own
#include "shader.h"
#include "Character.h"
#include "threadProcessImage1.h"

using namespace cv;
using namespace std;
using namespace glm;

string pathVideo;
string pathXML;
int width = 1024;
int height = 768;

glm::mat4 Projection;
glm::mat4 View;
glm::mat4 Model;

bool loadParameterCamera(string path, Mat& cameraMatrix, Mat& distCoeff) {
	FileStorage fs(path, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["Camera_Matrix"] >> cameraMatrix;
	fs["Distortion_Coefficients"] >> distCoeff;
	return true;
}

/*! @brief Esta funcion crea la estructura del objeto 3D.
*
*  @param[out] vao Vertex Array Object del objeto 3D que se creara.
*  @param[out] vboVertex Vertex Buffer Object de los vertices del objeto.
*  @param[out] vboTexture Vertex Buffer Object de los colores del objeto.
*/
void createCube(float objSize, GLuint &m_vao, GLuint &m_vboVertex, GLuint &m_vboTexture)
{
	objSize *= 0.5;

	// A cube has 6 faces with 2 triangles each, so this makes 6*2=12 triangles, and 12*3 vertices	
	static const GLfloat g_vertex_buffer_data[] = {
		-objSize, -objSize, -objSize,
		-objSize, -objSize, objSize,
		-objSize, objSize, objSize,
		objSize, objSize, -objSize,
		-objSize, -objSize, -objSize,
		-objSize, objSize, -objSize,
		objSize, -objSize, objSize,
		-objSize, -objSize, -objSize,
		objSize, -objSize, -objSize,
		objSize, objSize, -objSize,
		objSize, -objSize, -objSize,
		-objSize, -objSize, -objSize,
		-objSize, -objSize, -objSize,
		-objSize, objSize, objSize,
		-objSize, objSize, -objSize,
		objSize, -objSize, objSize,
		-objSize, -objSize, objSize,
		-objSize, -objSize, -objSize,
		-objSize, objSize, objSize,
		-objSize, -objSize, objSize,
		objSize, -objSize, objSize,
		objSize, objSize, objSize,
		objSize, -objSize, -objSize,
		objSize, objSize, -objSize,
		objSize, -objSize, -objSize,
		objSize, objSize, objSize,
		objSize, -objSize, objSize,
		objSize, objSize, objSize,
		objSize, objSize, -objSize,
		-objSize, objSize, -objSize,
		objSize, objSize, objSize,
		-objSize, objSize, -objSize,
		-objSize, objSize, objSize,
		objSize, objSize, objSize,
		-objSize, objSize, objSize,
		objSize, -objSize, objSize
	};

	// One color for each vertex. They were generated randomly.
	static const GLfloat g_color_buffer_data[] = {
		0.583f,  0.771f,  0.014f,
		0.609f,  0.115f,  0.436f,
		0.327f,  0.483f,  0.844f,
		0.822f,  0.569f,  0.201f,
		0.435f,  0.602f,  0.223f,
		0.310f,  0.747f,  0.185f,
		0.597f,  0.770f,  0.761f,
		0.559f,  0.436f,  0.730f,
		0.359f,  0.583f,  0.152f,
		0.483f,  0.596f,  0.789f,
		0.559f,  0.861f,  0.639f,
		0.195f,  0.548f,  0.859f,
		0.014f,  0.184f,  0.576f,
		0.771f,  0.328f,  0.970f,
		0.406f,  0.615f,  0.116f,
		0.676f,  0.977f,  0.133f,
		0.971f,  0.572f,  0.833f,
		0.140f,  0.616f,  0.489f,
		0.997f,  0.513f,  0.064f,
		0.945f,  0.719f,  0.592f,
		0.543f,  0.021f,  0.978f,
		0.279f,  0.317f,  0.505f,
		0.167f,  0.620f,  0.077f,
		0.347f,  0.857f,  0.137f,
		0.055f,  0.953f,  0.042f,
		0.714f,  0.505f,  0.345f,
		0.783f,  0.290f,  0.734f,
		0.722f,  0.645f,  0.174f,
		0.302f,  0.455f,  0.848f,
		0.225f,  0.587f,  0.040f,
		0.517f,  0.713f,  0.338f,
		0.053f,  0.959f,  0.120f,
		0.393f,  0.621f,  0.362f,
		0.673f,  0.211f,  0.457f,
		0.820f,  0.883f,  0.371f,
		0.982f,  0.099f,  0.879f
	};

	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);

	glGenBuffers(1, &m_vboVertex);
	glBindBuffer(GL_ARRAY_BUFFER, m_vboVertex);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);
	glVertexAttribPointer(
		0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
	);

	glGenBuffers(1, &m_vboTexture);
	glBindBuffer(GL_ARRAY_BUFFER, m_vboTexture);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_color_buffer_data), g_color_buffer_data, GL_STATIC_DRAW);
	glVertexAttribPointer(
		1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
		3,                                // size
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
	);
}

bool configEnvironment()
{
	// Initialise GLFW
	if (!glfwInit())
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return false;
	}

	//Anti-aliasing x4
	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);// opengl 4.4
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);// opengl 4.4
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);// opengl 4.4 version core (without retro-compatibility)

																  // Open a window and create its OpenGL context
	window = glfwCreateWindow(width, height, "Augmented Reality", NULL, NULL);
	if (window == NULL) {
		fprintf(stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n");
		glfwTerminate();
		return false;
	}
	glfwMakeContextCurrent(window);

	// Initialize GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		return false;
	}

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

	// Color background
	glClearColor(0.2f, 0.2f, 0.2f, 0.0f);

	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS);
	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);

	return true;
}

/*! @brief Esta funcion crea una textura a partir de un frame del tipo Mat de OpenCV.
*
*  @param[in] frame Frame o imagen de tipo Mat que será convertido en textura.
*  @param[out] vao Vertex Array Object del rectangulo creado para ser background.
*  @param[out] vboVertex Vertex Buffer Object de los vertices del rectangulo.
*  @param[out] vboTexture Vertex Buffer Object de los puntos de textura del rectangulo.
*/
GLuint createBackgroundFromFrame(Mat *frame, GLuint &m_vao, GLuint &m_vboVertex, GLuint &m_vboTexture)
{
	static const GLfloat g_vertex_buffer_data[] = {
		-1.0f,  1.0f, 0.0f,
		-1.0f, -1.0f, 0.0f,
		1.0f,  1.0f, 0.0f,

		-1.0f, -1.0f, 0.0f,
		1.0f, -1.0f, 0.0f,
		1.0f,  1.0f, 0.0f
	};
	static const GLfloat g_color_buffer_data[] = {
		0.0f, 0.0f,
		0.0f, 1.0f,
		1.0f, 0.0f,

		0.0f, 1.0f,
		1.0f, 1.0f,
		1.0f, 0.0f
	};

	GLuint textureID;
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame->cols, frame->rows, 0, GL_BGR, GL_UNSIGNED_BYTE, (void*)frame->ptr());
	glGenerateMipmap(GL_TEXTURE_2D);

	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);

	glGenBuffers(1, &m_vboVertex);
	glBindBuffer(GL_ARRAY_BUFFER, m_vboVertex);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

	glGenBuffers(1, &m_vboTexture);
	glBindBuffer(GL_ARRAY_BUFFER, m_vboTexture);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_color_buffer_data), g_color_buffer_data, GL_STATIC_DRAW);

	return textureID;
}

/*! @brief Esta funcion crea un background a partir de la textura dada como parámetro.
*
*  El background es generado en la posicion FAR de la cámara, es decir, que si el far es 100,
*  de la posicìón actual de la cámara + 100 es lo máximo que se puede visualizar.
*  Un objeto ubicado a distancia mayor a ésta no se visualizará en la cámara.
*
*  @param[in] textureID El ID de la textura cargada.
*  @param[in] vao Vertex Array Object del rectangulo asignado como background.
*  @param[in] vboVertex Vertex Buffer Object de los vertices del rectangulo.
*  @param[in] vboTexture Vertex Buffer Object de los puntos de textura para el rectangulo.
*/
void drawBackground(GLuint textureID, GLuint vao, GLuint vboVertex, GLuint vboTexture)
{
	glBindVertexArray(vao);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureID);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vboVertex);
	glVertexAttribPointer(
		0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
	);
	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, vboTexture);
	glVertexAttribPointer(
		1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
		2,                                // size
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
	);

	// Draw the triangle !
	glDrawArrays(GL_TRIANGLES, 0, 6); // 3 indices starting at 0 -> 1 triangle

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
}

void updateModelMatrix(Mat R, Mat t)
{
	Model = mat4(
		R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0),
		-R.at<double>(1, 0), -R.at<double>(1, 1), -R.at<double>(1, 2), -t.at<double>(1),
		-R.at<double>(2, 0), -R.at<double>(2, 1), -R.at<double>(2, 2), -t.at<double>(2),
		0, 0, 0, 1);
	Model = transpose(Model);
}

/*! @brief Esta función se encarga de transformar la matriz de OpenCV a una matriz de OpenGL.
*/
void matrixOpencv2Opengl(CameraParams camParams)
{
	width = camParams.imgWidth;
	//height = camParams.imgHeight;
	height = 480;
	int w = camParams.imgWidth, h = camParams.imgHeight;
	float fp = 5000.0f;	// far distance
	float np = 1.0f;	// near distance
	double fx = camParams.cameraMatrix.at<double>(0, 0),//intrinsicParams[0],
		sk = camParams.cameraMatrix.at<double>(0, 1),
		cx = camParams.cameraMatrix.at<double>(0, 2),//intrinsicParams[2],
		fy = camParams.cameraMatrix.at<double>(1, 1),//intrinsicParams[1],
		cy = camParams.cameraMatrix.at<double>(1, 2);//intrinsicParams[3];

	cout << "\nSizeImage: " << w << "/" << h << endl;
	/*
	Projection = mat4(1.0f);
	Projection[0] = vec4(2 * fx / w, 2 * sk / w, 2 * cx / w - 1, 0);
	Projection[1] = vec4(0, 2 * fy / h, 2 * cy / h - 1, 0);
	Projection[2] = vec4(0, 0, -(fp + np) / (fp - np), -2 * fp * np / (fp - np));
	Projection[3] = vec4(0, 0, -1, 0);
	*/
	Projection = mat4(1.0f);
	Projection[0] = vec4(2 * fx / w, 2 * sk / w, 1 - 2 * cx / w, 0);
	Projection[1] = vec4(0, 2 * fy / h, 2 * cy / h - 1, 0);
	Projection[2] = vec4(0, 0, -(fp + np) / (fp - np), -2 * fp * np / (fp - np));
	Projection[3] = vec4(0, 0, -1, 0);

	// Se aplica la transpuesta para generar matriz con col-major order
	Projection = glm::transpose(Projection);

	cout << "\nNueva matriz CV2GL\n";
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++)
			cout << Projection[i][j] << " ";
		cout << endl;
	}
	cout << endl;

	// Camera matrix
	View = glm::lookAt(
		glm::vec3(0, 0, 1), // Camera is at (x,y,z), in World Space
		glm::vec3(0, 0, 0), // and looks at the origin
		glm::vec3(0, 1, 0)	// looks at axis Y
	);
}

/*! @brief Esta función se encarga de dibujar el cubo en el screen.
*
*  @param[in] shaderID	ID del shader que sera utilizado para graficar el cubo.
*  @param[in] position	Posicion donde se colocara el objeto.
*  @param[in] vao		Vertex Array Object del objeto.
*/
void drawCube(ShaderInfo shaderInfo, vec3 position, GLuint vao)
{
	glUseProgram(shaderInfo.shaderID);
	glm::mat4 TranslationMatrix = translate(mat4(), position);
	glm::mat4 ModelView = Model * TranslationMatrix;
	glUniformMatrix4fv(shaderInfo.uniformInfo[0], 1, GL_FALSE, &ModelView[0][0]);
	glUniformMatrix4fv(shaderInfo.uniformInfo[1], 1, GL_FALSE, &View[0][0]);
	glUniformMatrix4fv(shaderInfo.uniformInfo[2], 1, GL_FALSE, &Projection[0][0]);

	glBindVertexArray(vao);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);

	glDrawArrays(GL_TRIANGLES, 0, 12 * 3);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
}

void drawCubeWithScale(ShaderInfo shaderInfo, vec3 position, GLuint vao, double scX, double scY, double scZ)
{
	glUseProgram(shaderInfo.shaderID);
	glm::mat4 TranslationMatrix = translate(mat4(), position);
	glm::mat4 ScalingMatrix = scale(mat4(), glm::vec3(scX, scY, scZ));
	glm::mat4 ModelView = Model * TranslationMatrix * ScalingMatrix;

	glUniformMatrix4fv(shaderInfo.uniformInfo[0], 1, GL_FALSE, &ModelView[0][0]);
	glUniformMatrix4fv(shaderInfo.uniformInfo[1], 1, GL_FALSE, &View[0][0]);
	glUniformMatrix4fv(shaderInfo.uniformInfo[2], 1, GL_FALSE, &Projection[0][0]);

	glBindVertexArray(vao);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);

	glDrawArrays(GL_TRIANGLES, 0, 12 * 3);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
}


void drawMaze(ShaderInfo shaderInfo, GLuint m_vao, int n, int m, double radio, double distCen, double objectSize) {
	double p = distCen - 2 * radio;
	double scalX = p / objectSize;
	double scalY = p / objectSize;
	double scalZ = 0.3;
	double aum = radio + p / 2;
	glm::vec3 pos;
	int flag = 0;
	for (int i = 0; i < 2 * n - 1; i++) {
		if (i % 2 != 0) {
			pos = vec3(0, ((i - 1) / 2)*distCen + aum, 0);
			glEnable(GL_BLEND);
			drawCubeWithScale(shaderInfo, pos, m_vao, 1, scalY, scalZ);
			glDisable(GL_BLEND);

			pos = vec3((m - 1)*distCen, ((i - 1) / 2)*distCen + aum, 0);
			glEnable(GL_BLEND);
			drawCubeWithScale(shaderInfo, pos, m_vao, 1, scalY, scalZ);
			glDisable(GL_BLEND);
		}

		else {
			for (int j = 0; j < 2 * m - 1; j++) {
				if (j % 2 == 0) {
					pos = vec3(j / 2 * distCen, i / 2 * distCen, 0);
					glEnable(GL_BLEND);
					drawCubeWithScale(shaderInfo, pos, m_vao, 1, 1, scalZ);
					glDisable(GL_BLEND);
				}
				else if (j == 1) {
					if (flag == 1) {
						pos = vec3((j - 1) / 2 * distCen + aum, i / 2 * distCen, 0);
						glEnable(GL_BLEND);
						drawCubeWithScale(shaderInfo, pos, m_vao, scalX, 1, scalZ);
						glDisable(GL_BLEND);
					}
				}
				else if (j == 2 * m - 3) {
					if (flag == 0) {
						pos = vec3((j - 1) / 2 * distCen + aum, i / 2 * distCen, 0);
						glEnable(GL_BLEND);
						drawCubeWithScale(shaderInfo, pos, m_vao, scalX, 1, scalZ);
						glDisable(GL_BLEND);
					}
				}
				else {
					pos = vec3((j - 1) / 2 * distCen + aum, i / 2 * distCen, 0);
					glEnable(GL_BLEND);
					drawCubeWithScale(shaderInfo, pos, m_vao, scalX, 1, scalZ);
					glDisable(GL_BLEND);
				}
			}
			flag = flag ? 0 : 1;
		}
	}
}

void justReadFromCamara() {
	int indexCamera = 0;
	cv::VideoCapture cap(indexCamera); // open the video camera no. 0
	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the video cam" << endl;
	}
	else {
		double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
		double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

		cout << "Frame size : " << dWidth << " x " << dHeight << endl;

		cv::namedWindow("MyVideo", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

		while (1)
		{
			cv::Mat frame;
			cout << "alto x ancho " << frame.cols << " " << frame.rows << endl;

			bool bSuccess = cap.read(frame); // read a new frame from video

			if (!bSuccess) //if not success, break loop
			{
				cout << "Cannot read a frame from video stream" << endl;
				break;
			}
			cout << "llego aca " << endl;
			cv::imshow("MyVideo", frame); //show the frame in "MyVideo" window

			if (cv::waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
			{
				cout << "esc key is pressed by user" << endl;
				break;
			}
		}
	}

}

glm::vec3 rotationMatrixToEulerAngles(Mat &R)
{
	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
	bool singular = sy < 1e-6; 
	float x, y, z;
	if (!singular)
	{
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else
	{
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
	return glm::vec3(x, y, z);
}


double genVelocityByTheta(double staX, double rangePer, double x, double vecX, int flag) 
{
	double res = 0;
	if (flag) {
		if (abs(x) < staX - rangePer)
			res = vecX * (x < 0 ? -1 : 1);
	}
	else {
		if (abs(x) > staX + rangePer)
			res = vecX * ( x > 0 ? 1 : -1);
	}
	return res;
}

int proccessRingVideo() {

	//string pathVideo = "data/PadronAnillos_03.avi";
	//string pathXML = "data/XML/params_A3.xml";
	//string pathXML = "data/XML/params_A6.xml";

	string pathVideo = "data/newvideo.wmv";
	string pathXML = "data/XML/short.xml";

	//float objSize = 45;//params.distanceKeypoints;
	float objSize = 9.5;//params.distanceKeypoints;

	Mat frame, cameraMatrix, distCoeff, tmpFrame;

	CameraParams params = readCameraParameters(pathXML);
	if (!params.isLoaded) {
		cout << "No se pudieron cargar los parametros de la camara" << endl;
		while (true);
		return -1;
	}

	params.distanceKeypoints = 41.5;
	matrixOpencv2Opengl(params);  // OJO

	/*bool flagLoad = loadParameterCamera(pathParams, cameraMatrix, distCoeff);
	if (flagLoad) {
		cout << " camera matrix " << cameraMatrix << endl;
		cout << " dist coeff " << distCoeff << endl;
	}*/

	if (!configEnvironment())
		return -1;

	//VideoCapture video;
	//video.open(pathVideo);

	/* REALSENSE BEGIN */
	VideoCapture video(0);
	if (!video.isOpened()) {
		cout << "No se cargo el video\n";
		while (true);
		return -1;
	}
	/* REALSENSE E*/

	ShaderInfo cubeShader;
	cubeShader.shaderID = LoadShaders("shaders/cubeVertShader.vert", "shaders/cubeFragShader.frag");
	cubeShader.uniformInfo.push_back(glGetUniformLocation(cubeShader.shaderID, "M"));
	cubeShader.uniformInfo.push_back(glGetUniformLocation(cubeShader.shaderID, "V"));
	cubeShader.uniformInfo.push_back(glGetUniformLocation(cubeShader.shaderID, "P"));

	// Cargando shaders para los objetos 3D
	ShaderInfo modelShader;
	modelShader.shaderID = LoadShaders("shaders/modelVertShader.vert", "shaders/modelFragShader.frag");
	modelShader.uniformInfo.push_back(glGetUniformLocation(modelShader.shaderID, "M"));
	modelShader.uniformInfo.push_back(glGetUniformLocation(modelShader.shaderID, "V"));
	modelShader.uniformInfo.push_back(glGetUniformLocation(modelShader.shaderID, "P"));
	modelShader.uniformInfo.push_back(glGetUniformLocation(modelShader.shaderID, "myTexture"));

	// Create and compile our GLSL program from the shaders
	GLuint backgroundProgramID = LoadShaders("shaders/backgroundVertShader.vert", "shaders/backgroundFragShader.frag");
	GLuint b_vao, b_vboVertex, b_vboTexture, b_textureID;
	GLuint textureID = glGetUniformLocation(backgroundProgramID, "myTexture");

	ShaderInfo sphereShader;
	sphereShader.shaderID = LoadShaders("shaders/sphereVertShader.vert", "shaders/sphereFragShader.frag");
	sphereShader.uniformInfo.push_back(glGetUniformLocation(sphereShader.shaderID, "M"));
	sphereShader.uniformInfo.push_back(glGetUniformLocation(sphereShader.shaderID, "V"));
	sphereShader.uniformInfo.push_back(glGetUniformLocation(sphereShader.shaderID, "P"));
	sphereShader.uniformInfo.push_back(glGetUniformLocation(sphereShader.shaderID, "myTexture"));


	// Creacion de objetos en el ambiente
	//--- Creacion del cubo
	GLuint m_vao, m_vboVertex, m_vboTexture;
	createCube(objSize, m_vao, m_vboVertex, m_vboTexture);

	//--- Creacion de los objetos 3D
	ModelData dataRedBird, dataPigNormal, dataSphere;
	Character redBird, pigNormal, redBird2, pigNormal2, glassSphere;
	dataRedBird.loadData("data/models/BirdRed.obj", "data/models/images/BirdRed.png");
	dataPigNormal.loadData("data/models/PigNormal.obj", "data/models/images/PigNormal.png");
	dataSphere.loadData("data/models/sphere.obj", "data/models/images/basketBall.png");
	redBird.init(&dataRedBird);
	redBird2.init(&dataRedBird);
	pigNormal.init(&dataPigNormal);
	pigNormal2.init(&dataPigNormal);
	glassSphere.init(&dataSphere);

	Mat currFrame;
	clock_t timeTotal = 0;
	int nFrameProccess = 0;
	vec3 cubePos, objPos;

	int id = 0;
	//clock_t time = clock();
	vector<Point3f> vectorDistReal = calcDistanceInWorldThreadRing();
	for (int i = 0; i < vectorDistReal.size(); i++) {
		cout << i << " " << vectorDistReal[i] << " == ";
	}
	int idFr = 0;	

	Size size(640, 480);

	int n = 3;
	int m = 4;
	double radio = 9.5;
	double distCent = 41.5;
	double objectSize = objSize;
	double pTemp = distCent - 2 * radio;

	glassSphere.setPosition(glm::vec3((m - 1)*distCent - pTemp / 2, (n - 1)*distCent - pTemp / 2, 0));

	do {
		/*PARA LEER FRAMES DE UN VIDEO - BEGIN*/
		//video >> frame;
		//if (frame.empty()) {
		//	cout << "No existe el frame\n";
		//	break;
		//}
		/*PARA LEER FRAMES DE UN VIDEO - END*/

		/*PARA CAPTURAR DESDE EL REAL SENSE - BEGIN*/
		bool bSucces = video.read(frame);
		if(!bSucces)
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}
		//imshow("Myvideo", frame);
		resize(frame, frame, size);
		if (cv::waitKey(30) == 27)
		{
			cout << "xq no funciona esta mierda " << endl;
		}
		/*PARA CAPTURAR DESDE EL REAL SENSE - END*/
	
		// Clear screen for background
		glClear(GL_COLOR_BUFFER_BIT);
		glUseProgram(backgroundProgramID);
		b_textureID = createBackgroundFromFrame(&frame, b_vao, b_vboVertex, b_vboTexture);
		glUniform1i(textureID, 0);
		drawBackground(b_textureID, b_vao, b_vboVertex, b_vboTexture);


	strucRotTrasVec1 resRotTra = getRotTrasByFrameRing1(frame, params.cameraMatrix, params.distCoeffs, vectorDistReal, idFr);
	cout << "frame " << idFr++ << endl;
	cout << "Vector de traslacion " << resRotTra.tvec << endl;
	cout << "Matriz de rotacion " << resRotTra.rMat3x3 << endl << endl;

	

	//	imshow("video ", frame);
		if (!resRotTra.frameValid)
			continue;
		updateModelMatrix(resRotTra.rMat3x3, resRotTra.tvec);

		glm::vec3 anglesEuler = rotationMatrixToEulerAngles(resRotTra.rMat3x3);
		cout << "angulos " << anglesEuler.x << " " << anglesEuler.y << " " << anglesEuler.z << endl;

		glm::vec3 velocSphere(0.0, 0.0, 0.0);

		double staY = 0.0;
		double staX = 3.1416;
		double staZ = 0.0;
		double rangePerY = 0.08;
		double rangePerX = 0.15;

		double vecX = 0.5;
		double vecY = 0.5;

		velocSphere.y = genVelocityByTheta(staX, rangePerX, anglesEuler.x, vecX, 1);
		velocSphere.x = genVelocityByTheta(staY, rangePerY, anglesEuler.y, vecY, 0);

	//////////////////////////////////////////////////////////////////////
	// Clear screen for 3D object
		glClear(GL_DEPTH_BUFFER_BIT);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		// Graficando los OBJETOS 3D
		//objPos = vec3(0, 0, 0);
		//redBird.setPosition(objPos);
		//redBird.draw(modelShader);
		//glEnable(GL_BLEND);
		//drawCube(cubeShader, objPos, m_vao);
		//glDisable(GL_BLEND);

		//objPos = vec3(0, params.distanceKeypoints * 2, 0);
		//pigNormal.setPosition(objPos);
		//pigNormal.draw(modelShader);
		//glEnable(GL_BLEND);
		//drawCube(cubeShader, objPos, m_vao);
		//glDisable(GL_BLEND);

		//objPos = vec3(params.distanceKeypoints * 3, params.distanceKeypoints * 2, 0);
		//pigNormal2.setPosition(objPos);
		//pigNormal2.draw(modelShader);
		//glEnable(GL_BLEND);
		//drawCube(cubeShader, objPos, m_vao);
		//glDisable(GL_BLEND);

		//objPos = vec3(params.distanceKeypoints * 3, 0, 0);
		//redBird2.setPosition(objPos);
		//redBird2.draw(modelShader);
		//glEnable(GL_BLEND);
		//drawCube(cubeShader, objPos, m_vao);
		//glDisable(GL_BLEND);

		drawMaze(cubeShader, m_vao, n, m, radio, distCent, objectSize);

		glEnable(GL_BLEND);
		//glassSphere.setPosition(glm::vec3( (m-1)*distCent - pTemp/2, (n-1)*distCent - pTemp/2, 0));
		double scal = 10;
		glassSphere.setScale(vec3(scal, scal, scal));		
		glassSphere.move(velocSphere);
		glassSphere.draw(sphereShader);
		glDisable(GL_BLEND);

		cout << endl << endl;
		
		// Swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();

		if (resRotTra.frameValid) {
			nFrameProccess++;
		}
		currFrame.release();
	//	waitKey(1);
	//	idFr++;
	} // Check if the ESC key was pressed or the window was closed
	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
		glfwWindowShouldClose(window) == 0);

	video.release();

	// Cleanup VBO
	glDeleteBuffers(1, &b_vboVertex);
	glDeleteBuffers(1, &b_vboTexture);
	glDeleteVertexArrays(1, &b_vao);
	glDeleteProgram(modelShader.shaderID);
	glDeleteProgram(cubeShader.shaderID);
	glDeleteProgram(backgroundProgramID);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();



	//time = clock() - time;
	//printf("Time ==> %f\n", ((float)time) / CLOCKS_PER_SEC);

	cv::waitKey(0);
	return 0;
}

int main() {

	//justReadFromCamara();
	proccessRingVideo();


	return 0;
}

