#pragma once
#include "globalConstants.h"
#include "Statistics.h"
#include "KFTracking1.h"
#include "TrackingGrid1.h"
#include <opencv2\opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <set>
#include <stack>
#include <queue>
#include <sstream>

//using namespace std;

template<typename Te>
string num2str(Te x) { stringstream ss; ss << x; return ss.str(); }
int numeroGlobal1; 

using namespace cv;
using namespace std;
using namespace stats;

struct strucRotTrasVec1 {				
	bool frameValid;
	Mat rMat3x3;
	Mat tvec;
};

struct CameraParams {
	int numFrames,
		imgWidth, imgHeight,
		numCols, numRows;
	float distanceKeypoints;
	Mat cameraMatrix;
	Mat distCoeffs;
	bool isLoaded;
};

CameraParams readCameraParameters(string path)
{
	CameraParams params;
	params.isLoaded = false;
	FileStorage fs(path, FileStorage::READ);
	if (!fs.isOpened()) {
		cout << "Archivo XML no encontrado\n";
		return params;
	}

	fs["num_of_frames"] >> params.numFrames;
	fs["image_width"] >> params.imgWidth;
	fs["image_height"] >> params.imgHeight;
	fs["num_cols"] >> params.numCols;
	fs["num_rows"] >> params.numRows;
	fs["distance_centers"] >> params.distanceKeypoints;
	fs["Camera_Matrix"] >> params.cameraMatrix;
	fs["Distortion_Coefficients"] >> params.distCoeffs;
	fs.release();
	params.isLoaded = true;
	return params;
}

pair<float, float> centNextPred;
TrackingGrid1* trackGrid;
Mat imgFrame;

float calcDistance2(Point2f p1, Point2f p2) {
	float dx = p1.x - p2.x;
	float dy = p1.y - p2.y;
	return sqrt((dx * dx) + (dy * dy));
}

bool cmpThreadRing(pair<int, int> p1, pair<int, int> p2) {
	if (p1.first != p2.first)
		return p1.first<p2.first;
	return p1.second > p2.second;
}

bool cmp2ThreadRing(pair<float, Point2f>  p1, pair<float, Point2f> p2) {
	return p1.first < p2.first;
}

// Comparador para el ordenado y poner numeros
bool cmp3ThreadRing(pair<float, float>  p1, pair<float, float> p2) {
	if (p1.second != p2.second)
		return p1.second>p2.second;
	return p1.first > p2.first;
}

void adaptiveThresholdIntegralImageForThreadRing1(Mat &input) {	
	// Ancho y altura de la imagen
	int w = input.cols;
	int h = input.rows;
	// Tamaño de la ventana S = (w/DIV_S)
	int s2 = (w / TH_DIV_S) / 2;
	// Declaracion de variables auxiliares
	int sum = 0;
	int count = 0;
	int x1, x2, y1, y2;

	// Imagen integral	
	unsigned long** intImg = new unsigned long*[h];
	for (int i = 0; i < h; i++) {
		intImg[i] = new unsigned long[w];
	}

	// Calculo de la imagen integral basado en los valores de los pixeles de input
	for (int i = 0; i < h; i++) {
		sum = 0;
		for (int j = 0; j < w; j++) {
			sum += input.at<uchar>(i, j);
			intImg[i][j] = i == 0 ? sum : intImg[i - 1][j] + sum;
		}
	}

	//// Se aplica thresholding y se obtiene la imagen binaria
	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			// Valores (x1,y1) y (x2,y2) de la ventana SxS
			x1 = j - s2;
			x2 = j + s2;
			y1 = i - s2;
			y2 = i + s2;

			// Verificación de bordes
			if (x1 < 0) x1 = 0;
			if (x2 >= w) x2 = w - 1;
			if (y1 < 0) y1 = 0;
			if (y2 >= h) y2 = h - 1;

			count = (x2 - x1) * (y2 - y1);
			sum = intImg[y2][x2] - intImg[y1][x2] - intImg[y2][x1] + intImg[y1][x1];

			// Proceso de binarización
			//input.at<uchar>(i, j) = (input.at<uchar>(i, j) * count) <= (sum * (1.0 - TH_T)) ? 255 : 0;
			input.at<uchar>(i, j) = (input.at<uchar>(i, j) * count) <= (sum * (1.0 - TH_T)) ? 0 : 255;
		}
	}
	for (int i = 0; i < h; i++) {
		delete intImg[i];		
	}
	delete intImg;
}


vector<Point2f> cleanNoiseCentersForThreadRing(vector<Point2f> vCenters, vector<pair<float, int> > vRadius, int maxError)
{
	double radioOptimo = 0.0;
	// Si el numero de centros es el mismo numero de componentes del patron, se regresa el mismo vector
	if (vCenters.size() <= (NUM_COL_ANILL * NUM_ROW_ANILL + maxError)) {
		// Ordenamiento de los radios en orden descendente para contar las frecuencias por intervalo
		sort(vRadius.rbegin(), vRadius.rend());
		radioOptimo = (vRadius[0].first + vRadius[vRadius.size() - 1].first) * 0.5;
		return vCenters;
	}

	vector<pair<int, float > > freqs;
	vector<pair<pair<float, float>, pair<int, int> > > extraInfo;
	float avgVal, stdVal;
	int modeVal, posMode;

	// Se obtiene las frecuencias de los datos agrupados
	getFrequences<float, int>(vRadius, freqs, extraInfo, false);
	// Se obtiene el promedio y la desviacion estandar
	getAvgStd(freqs, avgVal, stdVal);
	//// Se obtiene la moda y el intervalo en que se encuentra
	getMode(freqs, modeVal, posMode);

	// Vector donde se almacenaran los centros del patron
	vector<Point2f> keypoints;
	float minRad, maxRad;

	// Minimo y maximo radio que debe tener un circulo del patron
	if (modeVal == (int)(NUM_ROW_ANILL * NUM_COL_ANILL)) {
		minRad = extraInfo[posMode].first.first - PR_ERRORF;
		maxRad = extraInfo[posMode].first.second + PR_ERRORF;
	}
	else {
		minRad = freqs[posMode].second - stdVal;
		maxRad = freqs[posMode].second + stdVal;
	}
	for (size_t i = 0; i < vRadius.size(); i++) {
		if (vRadius[i].first >= minRad && vRadius[i].first <= maxRad) {
			keypoints.push_back(vCenters[vRadius[i].second]);
		}
	}
	return keypoints;
}

vector<Point2f> findROI_ringsForThreadRing1(Mat &image)
{
	// Obtencion de los contornos con jerarquia
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

	// Vector de centros
	vector<Point2f> keypoints;

	// Variables auxiliares
	double areaPar, auxFactorPar, auxFactorCurr;
	int parent, child;
	Point2f centerCurr, centerPar;

	vector<pair<float, int> > vectRadios;
	for (size_t i = 0; i < contours.size(); i++)
	{
		parent = hierarchy[i][3];
		child = hierarchy[i][2];

		if (child == -1) {
			if (parent != -1 && hierarchy[i][0] == -1 && hierarchy[i][1] == -1) {
				// PADRE: Rectangulo donde encaja la elipse o el contorno del padre
				RotatedRect boxPar;
				if (contours[parent].size() < CON_MIN_SIZE_CONTOUR)
					boxPar = minAreaRect(contours[parent]);
				else {
					Mat pointsf;
					Mat(contours[parent]).convertTo(pointsf, CV_32F);
					boxPar = fitEllipse(pointsf);
				}
				centerPar = boxPar.center;

				// ACTUAL: Rectangulo donde encaja la elipse o el contorno del actual
				RotatedRect boxCurr;
				if (contours[i].size() < CON_MIN_SIZE_CONTOUR)
					centerCurr = centerPar;
				else {
					Mat pointsf;
					Mat(contours[i]).convertTo(pointsf, CV_32F);
					boxCurr = fitEllipse(pointsf);
					centerCurr = boxCurr.center;
				}

				// Calculo de areas
				areaPar = contourArea(contours[parent]);

				// Factor de aspect ratio
				auxFactorPar = std::min(boxPar.size.width, boxPar.size.height) / std::max(boxPar.size.width, boxPar.size.height);
				auxFactorCurr = std::min(boxCurr.size.width, boxCurr.size.height) / std :: max(boxCurr.size.width, boxCurr.size.height);
				if (auxFactorPar < R_PAR_MIN_ASPECT_RATIO || auxFactorCurr < R_CHD_MIN_ASPECT_RATIO)
					continue;

				// Factor de rectangularidad
				auxFactorPar = areaPar / boxPar.size.area();
				if (auxFactorPar < R_PAR_MIN_RECTAN)
					continue;

				// Almacenamiento del centro de los anillos concentricos
				keypoints.push_back(Point2f((centerCurr.x + centerPar.x) * 0.5, (centerCurr.y + centerPar.y) * 0.5));
				vectRadios.push_back(make_pair(std::max(boxPar.size.width, boxPar.size.height) * 0.5, keypoints.size() - 1));
			}
		}
	}
	if (keypoints.size() > 0)
		keypoints = cleanNoiseCentersForThreadRing(keypoints, vectRadios, 2);
	return keypoints;
}

vector<Point2f> cleanNoiseUsingDistancesForThreadRing(vector<Point2f>& keypoints)
{
	int maximobolitas = NUM_ROW_ANILL * NUM_COL_ANILL;
	vector<Point2f> patternActual;

	if ((int)keypoints.size() <= maximobolitas) {
		return keypoints;
	}
	else {
		int factDist = 2;
		// Calculamos las distancias de uno vs todos
		vector<pair<float, pair<int, int> > > distances;
		float dist = OP_INF;
		int idx = -1;
		for (size_t i = 0; i < keypoints.size() - 1; i++) {
			dist = OP_INF;
			idx = -1;
			for (size_t j = i + 1; j < keypoints.size(); j++) {
				if (dist > calcDistance2(keypoints[i], keypoints[j])) {
					dist = calcDistance2(keypoints[i], keypoints[j]);
					idx = j;
				}
			}
			distances.push_back(make_pair(dist, make_pair(i, idx)));
		}
		// Calculamos la distancia media usando estadista por datos agrupados en 5 intervalos
		vector<pair<int, float> > freqs;
		vector<pair<pair<float, float>, pair<int, int> > > extraInfo;
		int posMode, modeVal;
		getFrequences<float, pair<int, int> >(distances, freqs, extraInfo, false, 5);
		getMode(freqs, modeVal, posMode);
		float maxVal = freqs[posMode].second * factDist;
		// Usando la moda eliminamos los puntos que se encuentren muy alejados
		set<pair<float, float> > pointsSelected;
		for (size_t i = 0; i < distances.size(); i++) {
			if (distances[i].first < maxVal) {
				pointsSelected.insert(make_pair(keypoints[distances[i].second.first].x, keypoints[distances[i].second.first].y));
				pointsSelected.insert(make_pair(keypoints[distances[i].second.second].x, keypoints[distances[i].second.second].y));
			}
		}
		// Insertamos los puntos en la variable patternActual para que sea evaluado en Kalman
		set<pair<float, float> >::iterator it;
		for (it = pointsSelected.begin(); it != pointsSelected.end(); ++it) {
			patternActual.push_back(Point2f((*it).first, (*it).second));
		}
	}

	//FILTRO DE KALMAN USANDO PREDICCION t+1	
	TrackingGrid1* trackGrid = new TrackingGrid1(maximobolitas);
	vector<pair<int, float> > vectorKeyDis;
	for (int i = 0; i<(int)patternActual.size(); i++) {
		float distx = centNextPred.first - patternActual[i].x;
		float disty = centNextPred.second - patternActual[i].y;
		vectorKeyDis.push_back(make_pair(i, distx*distx + disty*disty));
	}

	//vectorKeyDis = trackGrid->sortBySecond(vectorKeyDis);

	trackGrid->sortBySecond(vectorKeyDis);
	vector<Point2f> tempVector = patternActual;
	patternActual.clear();
	int nVect = (int)vectorKeyDis.size() >= maximobolitas ? maximobolitas : (int)vectorKeyDis.size();
	for (int i = 0; i < nVect; i++) {
		float posX = tempVector[vectorKeyDis[i].first].x;
		float posY = tempVector[vectorKeyDis[i].first].y;
		patternActual.push_back(Point2f(posX, posY));
	}

	// TRACKING GRID ANILLOS
	KFTracking1* kfTracking = new  KFTracking1(1);
	double dtKFTrac = 0.0;
	if ((int)patternActual.size() <= maximobolitas) {
		Point2f centroide = trackGrid->getCentroide(patternActual);

		vector<Mat>* arrayMat = new vector<Mat>();
		Mat tempKF(MEASURE_SIZE, 1, TYPE_KF);
		tempKF.at<float>(0) = centroide.x;
		tempKF.at<float>(1) = centroide.y;
		tempKF.at<float>(2) = (float)0.0;
		tempKF.at<float>(3) = (float)0.0;
		arrayMat->push_back(tempKF);

		//primera vez que aparece el patron
		if (kfTracking->firstFound[0]) {
			kfTracking->setStateInit(arrayMat);
			kfTracking->firstFound[0] = false;
		}
		else {
			kfTracking->predict(dtKFTrac);
			vector<Mat>* correct = kfTracking->kalmanCorrection(arrayMat);
			vector<Mat>* matForPre = kfTracking->futureNTime(1);
			centNextPred = make_pair((*matForPre)[0].at<float>(0), (*matForPre)[0].at<float>(1));
		}
	}
	delete trackGrid;
	delete kfTracking; 
	return patternActual;
}

bool trackingRingsPointsForThreadRing(vector<Point2f> &keypoints) {
	if (keypoints.size() != NUM_COL_ANILL * NUM_ROW_ANILL) {
		//cout << keypoints.size() << " != NUM_COL_ANILL * NUM_COL_ANILL " <<  endl;
		return false;
	}

	// En esta parte se empieza a utilizar el convexhull para hallar los segmentos de arriba y abajo
	if (keypoints.size() > 0) {

		vector<vector<Point2f> > keys;
		keys.push_back(keypoints);
		vector<vector<Point2f> > hull(1);
		convexHull(Mat(keypoints), hull[0], false);

		//Obteniendo las esquinas del convexhull en el patron
		vector<int> posCornes = trackGrid->getPosCornes(hull);
		if (posCornes.size() < 4)
			return false;
		vector<pair<Point2f, Point2f> > extremosUpDown;
		// Hallando extremos, arriba y abajo
		for (int i = 0; i < (int)posCornes.size(); i++) {
			extremosUpDown.push_back(make_pair(hull[0][posCornes[i]], hull[0][posCornes[(i + 1) % 4]]));
		}

		// Hallando una recta con 4 puntos en su contenido extremosUpDown
		vector<vector<pair<float, float> > > ans;
		for (int i = 0; i<(int)extremosUpDown.size(); i++) {
			Point2f A = extremosUpDown[i].first;
			Point2f B = extremosUpDown[i].second;
			Point2f P;
			// Interseccion de la recta AB con el punto P
			vector<pair<float, float> > aux;
			for (int k = 0; k<(int)keypoints.size(); k++) {

				// Vemos que no sean los mismo puntos para evitar overflow
				if ((keypoints[k].x == A.x && keypoints[k].y == A.y) || (keypoints[k].x == B.x && keypoints[k].y == B.y)) continue;
				P = keypoints[k];
				// Hallando la distancia del punto P a la recta AB
				double numerador = (P.x - A.x) * (B.y - A.y) - (P.y - A.y) * (B.x - A.x);
				double denominador = sqrt((B.x - A.x)*(B.x - A.x) + (B.y - A.y)*(B.y - A.y));
				double distancia = numerador / denominador;
				if (abs((int)distancia) < 6) { // se escoge 6 como tolerancia de precision
					aux.push_back(make_pair(keypoints[k].x, keypoints[k].y));
				}
			}
			aux.push_back(make_pair(A.x, A.y));
			aux.push_back(make_pair(B.x, B.y));

			if ((int)aux.size() == NUM_COL_ANILL) {
				//Ordenando Ascendentemente x, descendentemente y
				sort(aux.begin(), aux.end(), cmpThreadRing);
				ans.push_back(aux);
			}
		}

		//vector<pair<float, float> > SortPoints;
		//stack<vector<pair<float, float> > > pila;
		//// escribir lineas de colores
		//if (ans.size()>1) {

		//	Point2f PPP = Point2f(ans[0][0].first, ans[0][0].second);
		//	for (int j = 0; j<min((int)ans[0].size(), (int)ans[1].size()); j++) {

		//		SortPoints.push_back(make_pair(ans[0][j].first, ans[0][j].second));
		//		SortPoints.push_back(make_pair(ans[1][j].first, ans[1][j].second));

		//		vector<pair<float, Point2f> > distanciaRecta; // Distancia a la recta AB del punto P
		//													  // Hallando los puntos de la recta AB
		//		Point2f A = Point2f(ans[0][j].first, ans[0][j].second);
		//		Point2f B = Point2f(ans[1][j].first, ans[1][j].second);
		//		Point2f P;
		//		// Keypoints tiene todos los puntos del patron
		//		for (int k = 0; k<(int)keypoints.size(); k++) {
		//			//Vemos que no sean los mismo puntos para evitar overflow
		//			if ((keypoints[k].x == A.x && keypoints[k].y == A.y) || (keypoints[k].x == B.x && keypoints[k].y == B.y)) continue;
		//			P = keypoints[k];
		//			// Hallando la distancia del punto P a la recta AB
		//			double numerador = (P.x - A.x) * (B.y - A.y) - (P.y - A.y) * (B.x - A.x);
		//			double denominador = sqrt((B.x - A.x)*(B.x - A.x) + (B.y - A.y)*(B.y - A.y));
		//			double distancia = numerador / denominador;
		//			distanciaRecta.push_back(make_pair(abs((float)distancia), P));
		//		}

		//		// Ordenamos las distancias, para escoger los 3 mas cercanos
		//		sort(distanciaRecta.begin(), distanciaRecta.end(), cmp2ThreadRing);
		//		for (int i = 0; i<3; i++) {
		//			SortPoints.push_back(make_pair(distanciaRecta[i].second.x, distanciaRecta[i].second.y));
		//		}

		//		sort(SortPoints.rbegin(), SortPoints.rend(), cmp3ThreadRing);
		//		// Almacenando los puntos de una recta
		//		pila.push(SortPoints);
		//		SortPoints.clear();

		//		// Escribiendo linea para la siguiente columna del patron
		//		PPP = Point2f(ans[1][j].first, ans[1][j].second);
		//	}

		//	// Escribiendo las rectas de manera descendente
		//	int counter = 0;  // Contador para etiquetar los puntos
		//	keypoints.clear();
		//	// Extraendo los elementos de la pila
		//	while (!pila.empty()) {
		//		// Escribiendo numeros
		//		for (int i = 0; i<pila.top().size(); i++) {					
		//			keypoints.push_back(Point2f(pila.top()[i].first, pila.top()[i].second));
		//		}
		//		pila.pop();
		//	}
		//}

		/* CODE FRYZITO */
		vector<pair<float, float> > SortPoints;
		queue<vector<pair<float, float> > > cola;
		// escribir lineas de colores
		if (ans.size()>1) {
			Point2f PPP = Point2f(ans[0][0].first, ans[0][0].second);
			for (int j = 0; j<std::min((int)ans[0].size(), (int)ans[1].size()); j++) {
				SortPoints.push_back(make_pair(ans[0][j].first, ans[0][j].second));
				SortPoints.push_back(make_pair(ans[1][j].first, ans[1][j].second));

				vector<pair<float, Point2f> > distanciaRecta; // Distancia a la recta AB del punto P
															  // Hallando los puntos de la recta AB
				Point2f A = Point2f(ans[0][j].first, ans[0][j].second);
				Point2f B = Point2f(ans[1][j].first, ans[1][j].second);
				Point2f P;
				// Keypoints tiene todos los puntos del patron
				for (int k = 0; k<(int)keypoints.size(); k++) {
					//Vemos que no sean los mismo puntos para evitar overflow
					if ((keypoints[k].x == A.x && keypoints[k].y == A.y) || (keypoints[k].x == B.x && keypoints[k].y == B.y)) continue;
					P = keypoints[k];
					// Hallando la distancia del punto P a la recta AB
					double numerador = (P.x - A.x) * (B.y - A.y) - (P.y - A.y) * (B.x - A.x);
					double denominador = sqrt((B.x - A.x)*(B.x - A.x) + (B.y - A.y)*(B.y - A.y));
					double distancia = numerador / denominador;
					distanciaRecta.push_back(make_pair(abs((float)distancia), P));
				}

				// Ordenamos las distancias, para escoger los 3 mas cercanos
				sort(distanciaRecta.begin(), distanciaRecta.end(), cmp2ThreadRing);
				for (int i = 0; i < NUM_ROW_ANILL - 2; i++) { // Escogemos el que esta mas cerca al la recta AB
					SortPoints.push_back(make_pair(distanciaRecta[i].second.x, distanciaRecta[i].second.y));
				}
				sort(SortPoints.begin(), SortPoints.end(), cmp3ThreadRing);
				// Almacenando los puntos de una recta
				cola.push(SortPoints);
				SortPoints.clear();

				// Escribiendo linea para la siguiente columna del patron
				PPP = Point2f(ans[1][j].first, ans[1][j].second);
			}

			// Escribiendo las rectas de manera descendente
			int counter = 0;  // Contador para etiquetar los puntos
			keypoints.clear();
			// Extraendo los elementos de la pila
			while (!cola.empty()) { // pila.empty()
									// Escribiendo numeros
				for (int i = 0; i<cola.front().size(); i++) {
					keypoints.push_back(Point2f(cola.front()[i].first, cola.front()[i].second));
				}
				cola.pop();
			}
		}
		/* END CODE FRYZITO*/



		// PARA GRAFICAR LOS PUNTOS ENCONTRADOS
		/*vector<Scalar> colors;
		colors.push_back(MY_COLOR_GREEN);
		colors.push_back(MY_COLOR_ORANGE);
		colors.push_back(MY_COLOR_BLUE);
		colors.push_back(MY_COLOR_YELLOW);
		colors.push_back(MY_COLOR_WHITE);
		colors.push_back(MY_COLOR_RED);

		for (int i = 0; i < NUM_COL_ANILL; i++) {
			for (int j = 0; j < NUM_ROW_ANILL; j++) {
				int idKP = i * NUM_ROW_ANILL + j;
				stringstream sstr;
				sstr << idKP;
				cv::putText(imgFrame, sstr.str(), keypoints[idKP], cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 255), 2);
				circle(imgFrame, keypoints[idKP], 5, colors[i], CV_FILLED, 8, 0);
				if (idKP + 1 != (int)keypoints.size()) {										
					line(imgFrame, keypoints[idKP], keypoints[idKP+1], colors[i]);
				}
			}
		}
		imshow("etiquetado de los puntos ", imgFrame);*/
		//imwrite("C:\\Users\\user\\Pictures\\CompuGrafica\\frameVeloz\\Frames_" + num2str<int>(numeroGlobal) + ".png", imgFrame);
	}
	return true;
}

vector<Point3f> calcDistanceInWorldThreadRing()
{
	vector<Point3f> realCenters;
	float distanceKeyPoints = 41.5;
	for (size_t i = 0; i < NUM_COL_ANILL; ++i)
		for (size_t j = 0; j < NUM_ROW_ANILL; ++j)
			realCenters.push_back(Point3f(float(i * distanceKeyPoints), float(j * distanceKeyPoints), 0));
	return realCenters;
}

strucRotTrasVec1 getRotTrasByFrameRing1(Mat frame, Mat& cameraMatrix, Mat& distCoeffs, vector<Point3f>& objectsPoints, int nFrame) {
	numeroGlobal1 = nFrame;

	imgFrame = frame;

	vector<Point2f> keypoints;
	Mat rvec = Mat::zeros(Size(3, 1), CV_8UC3);
	Mat tvec = Mat::zeros(Size(3, 1), CV_8UC3);
	Mat rMat3x3 = Mat::zeros(Size(3, 3), CV_8UC3);
	Mat tmpFrame;
	strucRotTrasVec1 res;

	// PROCESANDO LA IMAGEN DEL FRAME DE ANILLOS - ELIMINACION DE RUIDO
	// Conversion de imagen a escala de grises
	cvtColor(frame, tmpFrame, CV_BGR2GRAY);
	// Aplicacion de filtro gaussiano
	GaussianBlur(tmpFrame, tmpFrame, Size(3, 3), 0.5, 0.5);
	//// Segmentacion de imagen usando threshold adaptativo	
	adaptiveThresholdIntegralImageForThreadRing1(tmpFrame);
	//// find roi rings
	keypoints = findROI_ringsForThreadRing1(tmpFrame);
	////// Obtención de los centros finales
	keypoints = cleanNoiseUsingDistancesForThreadRing(keypoints);	
	////// etiquetado
	bool trackCorrect = trackingRingsPointsForThreadRing(keypoints);
	if (!trackCorrect) {
		res.frameValid = false;
		return res;
	}	
	// generacion de los puntos reales	
	// obtencion de los vectores de rotacion y traslacion
	solvePnP(objectsPoints, keypoints, cameraMatrix, distCoeffs, rvec, tvec);
	// convierte el vector 3x1 a una matriz 3x3
	Rodrigues(rvec, rMat3x3);

	res.frameValid = true;
	res.tvec = tvec;
	res.rMat3x3 = rMat3x3;
	return res;
}