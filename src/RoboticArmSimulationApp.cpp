#include <math.h>

#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/CinderImGui.h"

/* Drawing graph size in pixels*/
#define GRAPH_SIZE 200
#define GRAPH_CHILD_SIZE 20

/* Size of one unit in pixels*/
#define GRAPH_USIZE 20

using namespace ci;
using namespace ci::app;
using namespace std;

class RoboticArmSimulationApp : public App {
public:
	RoboticArmSimulationApp()
		: pMouse(), mInputs() {};

private:
	vec2 pOrigin;			/* Absolute origin in pixels relative to world */
	vec2 pMouse;			/* Mouse position in pixels relative to world */
	vec2 pPointPosition;	/* Point position in pixels relative to world */
	bool isValid = false;	/* True when mouse is down */

	/* ImGui Inputs */
	struct Inputs {
		float arm_length;
		float arm2_length;

		Inputs() :
			arm_length(4),
			arm2_length(4)
		{};
	};

	Inputs mInputs;
	float theta_rad = 0.0f;
	float theta2_rad = 0.0f;
	float cos_theta = 0.0f;
	float sin_theta = 0.0f;

	vec2 c_origin;		/* Child origin in pixels relative to world */
	vec2 c_translation;	/* Child translation relative to origin*/
	vec2 c_rotation;	/* Child rotation relative to origin */

	vec2 uAbsolute;				/* Point position respect to absolute origin */
	vec2 uRelativeGraphical;	/* Point position relative to child origin calculated graphically */
	vec2 uRelativeGeometric;	/* Point position relative to child origin calculated geometrically */

public:
	/* Cinder Methods */
	void setup() override;
	void mouseDown(MouseEvent event) override;
	void mouseUp(MouseEvent event) override;
	void mouseDrag(MouseEvent event) override;
	void update() override;
	void draw() override;

private:
	float roundCos(const float _cos);
	float getAnglefromXY(float x, float y);
	float getTheta2fromXY(float x, float y, float L1, float L2);
	float getTheta1fromXY(float x, float y, float L1, float L2, float theta2);
	
	void computeIK();
};

void RoboticArmSimulationApp::setup()
{
	ImGui::Initialize();
}

void RoboticArmSimulationApp::mouseDown( MouseEvent event )
{
	pMouse = vec2(event.getX(), event.getY());
	isValid = true;
}

void RoboticArmSimulationApp::mouseUp(MouseEvent event)
{
	isValid = false;
}

void RoboticArmSimulationApp::mouseDrag(MouseEvent event)
{
	pMouse = vec2(event.getX(), event.getY());
}

void RoboticArmSimulationApp::update()
{
	if (!isValid)
		return;

	/* Calculate the absolute origin in pixels respect to world */
	pOrigin = getWindowCenter();

	/* Calculates point position respect to absolute origin (in graph units) */
	uAbsolute = (pMouse - pOrigin);
	uAbsolute = vec2(uAbsolute.x / GRAPH_USIZE, uAbsolute.y / GRAPH_USIZE);

	// TODO: Clean. 
	/* If radio is bigger than the maximum reach of the arms */
	if (sqrt(uAbsolute.x * uAbsolute.x + uAbsolute.y * uAbsolute.y) > (mInputs.arm_length + mInputs.arm2_length)) {
		float radio = (mInputs.arm_length + mInputs.arm2_length);
		float _cos = roundCos(uAbsolute.x / sqrt(uAbsolute.x * uAbsolute.x + uAbsolute.y * uAbsolute.y));
		float angle = acos(_cos);
		uAbsolute = vec2(radio * cos(angle), -radio * sin(angle));
	}

	/* Compute angles */
	computeIK();

	cos_theta = cos(theta_rad);
	sin_theta = sin(theta_rad);

	/* Compute child rotation, translation and origin respect to world, in pixels */
	c_rotation = vec2(GRAPH_CHILD_SIZE * cos_theta, GRAPH_CHILD_SIZE * sin_theta);
	c_translation = vec2(GRAPH_USIZE * mInputs.arm_length * cos_theta, -GRAPH_USIZE * mInputs.arm_length * sin_theta);
	c_origin = pOrigin + c_translation;

	/* Calculates point position respect to child origin graphically (in graph units) */
	uRelativeGraphical = (pMouse - c_origin);
	uRelativeGraphical = vec2(uRelativeGraphical.x / GRAPH_USIZE, uRelativeGraphical.y / GRAPH_USIZE);
	// TODO Use graphical long calculation
	uRelativeGraphical = vec2(uRelativeGraphical.x * cos_theta + (-uRelativeGraphical.y) * sin_theta,
		(uRelativeGraphical.y) * cos_theta + uRelativeGraphical.x * sin_theta);

	/* Calculates point position respect to child origin geometrically (in graph units) */
	/* Operations on the Y axis are sign inverted becasue of opengl's
	 * frame of remerence (Y pointing downwards) */
	uRelativeGeometric = vec2(
		(uAbsolute.x - mInputs.arm_length * cos_theta) * cos_theta - (uAbsolute.y + mInputs.arm_length * sin_theta) * sin_theta,
		(uAbsolute.y + mInputs.arm_length * sin_theta) * cos_theta + (uAbsolute.x - mInputs.arm_length * cos_theta) * sin_theta
	);

	pPointPosition = c_origin + vec2(GRAPH_USIZE * mInputs.arm2_length * cos(theta2_rad + theta_rad), -GRAPH_USIZE * mInputs.arm2_length * sin(theta2_rad + theta_rad));
}

void RoboticArmSimulationApp::draw()
{
	/* Clear screen */
	gl::clear(Color(1, 1, 1));
	
	/* Draw parent reference*/
	gl::color(Color(0, 0, 255));
	gl::drawLine(pOrigin, pOrigin + vec2(GRAPH_SIZE, 0));	// x axis
	gl::drawLine(pOrigin, pOrigin + vec2(0, -GRAPH_SIZE));	// y axis
	gl::color(Color(255, 0, 0));
	gl::drawStrokedCircle(pOrigin, GRAPH_USIZE * (mInputs.arm_length + mInputs.arm2_length));

	/* Draw child reference*/
	gl::color(Color(0, 0, 255));
	gl::drawLine(c_origin, c_origin + vec2(c_rotation.x, -c_rotation.y));	// x axis
	gl::drawLine(c_origin, c_origin + vec2(-c_rotation.y, -c_rotation.x));	// y axis

	/* Draw arms */
	gl::color(Color(0, 0, 0));
	gl::drawLine(pOrigin, c_origin);
	gl::drawLine(c_origin, pPointPosition);
	gl::drawSolidCircle(pPointPosition, 5);
	gl::drawSolidCircle(c_origin, 5);

	/* Draw ImGui*/
	/* Outputs */
	ImGui::Text("Absolute: %.2f, %.2f", uAbsolute.x, -uAbsolute.y);
	ImGui::Text("Relative: %.2f, %.2f", uRelativeGraphical.x, -uRelativeGraphical.y);
	ImGui::Text("Geometric: %.2f, %.2f", uRelativeGeometric.x, -uRelativeGeometric.y);

	/* Inputs */
	ImGui::SliderFloat("L1", &mInputs.arm_length, .0f, 10);
	ImGui::SliderFloat("L2", &mInputs.arm2_length, .0f, 10);
}

float RoboticArmSimulationApp::roundCos(const float _cos)
{
	if (_cos > 1.0f)
		return 1.0f;
	else if (_cos < -1.0f)
		return -1.0f;
	return _cos;
}

float RoboticArmSimulationApp::getAnglefromXY(float x, float y)
{
	return acos(x / sqrt(x * x + y * y));
}

float RoboticArmSimulationApp::getTheta2fromXY(float x, float y, float L1, float L2)
{
	/* Equation:
	* dcos(theta2) = f
	* d = 2 * L1 * L2
	* f = (x^2 + y^2) - (L1^2 + L2^2) */
	float d = 2 * L1 * L2;
	float f = (x * x + y * y) - (L1 * L1 + L2 * L2);
	float _cos = roundCos(f / d);
	return acos(_cos);
}

float RoboticArmSimulationApp::getTheta1fromXY(float x, float y, float L1, float L2, float theta2)
{
	/* Equation:
	* Acos(theta) - Bsin(theta) = E
	* Bcos(theta) + Asin(theta) = F
	* cos(theta) = (AE - BF) / (A^2 - B^2)
	* A = L1 + L2cos(theta2)
	* B = L2sin(theta2)
	* E = x, F = y */
	float A = L1 + L2 * cos(theta2);
	float B = L2 * sin(theta2);
	float _cos = roundCos((A * x + B * y) / (A * A + B * B));
	return acos(_cos);
}

void RoboticArmSimulationApp::computeIK()
{
	// TODO: Consider lower elbow cases and negative angles
	theta2_rad = -1 * getTheta2fromXY(uAbsolute.x, -uAbsolute.y, mInputs.arm_length, mInputs.arm2_length);
	theta_rad = getTheta1fromXY(uAbsolute.x, -uAbsolute.y, mInputs.arm_length, mInputs.arm2_length, theta2_rad);
}

CINDER_APP( RoboticArmSimulationApp, RendererGl )
