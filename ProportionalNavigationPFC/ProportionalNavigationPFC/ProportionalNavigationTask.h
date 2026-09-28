// guard header (header processed just once)
#ifndef PROPORTIONAL_NAVIGATION_TASK_H
#define PROPORTIONAL_NAVIGATION_TASK_H
#endif
// #pragma once


// Project Headers
#include <vector>
#include <iostream>


// Declarations
namespace pfc 
{

static const int s_GAIN_H = 650;
static const int s_GAIN_N = 3;

static const double s_FINAL_TIME = 20.0;
static const double s_STEP_TIME = 1E-3;

static constexpr double s_ACCELERATION_TARGET_X = 1.2;
static constexpr double s_ACCELERATION_TARGET_Y = -1.0;
static constexpr double s_ACCELERATION_TARGET_Z = 0.2;

class ProportionalNavigationTask
{
public:
	enum ECoordinates
	{
		X_COORDINATE = 0,
		Y_COORDINATE,
		Z_COORDINATE,
		STATE_DIM
	};

	struct SStateVector
	{
		double m_targetPositionX;
		double m_targetPositionY;
		double m_targetPositionZ;


		double m_interceptorPositionX;
		double m_interceptorPositionY;
		double m_interceptorPositionZ;


		double m_targetVelocityX;
		double m_targetVelocityY;
		double m_targetVelocityZ;


		double m_interceptorVelocityX;
		double m_interceptorVelocityY;
		double m_interceptorVelocityZ;
	};

	struct SDerivativeStateVector
	{
		double m_DeriTargetPositionX;
		double m_DeriTargetPositionY;
		double m_DeriTargetPositionZ;


		double m_DeriInterceptorPositionX;
		double m_DeriInterceptorPositionY;
		double m_DeriInterceptorPositionZ;


		double m_DeriTargetVelocityX;
		double m_DeriTargetVelocityY;
		double m_DeriTargetVelocityZ;


		double m_DeriInterceptorVelocityX;
		double m_DeriInterceptorVelocityY;
		double m_DeriInterceptorVelocityZ;
	};

	struct SPositionVelocityRelatives
	{
		double m_positionX;
		double m_positionY;
		double m_positionZ;

		double m_velocityX;
		double m_velocityY;
		double m_velocityZ;
	};

	struct SModules
	{
		double m_modulePosition;
		double m_moduleVelocity;
		double m_moduleAcceleration;

	};

	struct SAcceleartion
	{
		double m_targetAccelerationX;
		double m_targetAccelerationY;
		double m_targetAccelerationZ;

		double m_interceptorAccelerationX;
		double m_interceptorAccelerationY;
		double m_interceptorAccelerationZ;
	};

	SStateVector m_initialState, m_processIntermediateStateVector;

	SPositionVelocityRelatives m_processPoisitionVelocityRelatives;

	SModules m_processModules;

	double m_processLOS[3];

	SAcceleartion m_processAcceleration;

	SDerivativeStateVector m_processDerivativeS1, m_processDerivativeS2, m_processDerivativeS3, m_processDerivativeS4;

	const int m_totalSteps;

	ProportionalNavigationTask(SStateVector initialState);

	~ProportionalNavigationTask();

	void saveToCSV_throughVector(const std::string& filename);

	void saveToCSV_throughPointer(const char* filename);

private:
	
	std::vector<SStateVector> m_vStateVector;

	SStateVector* m_pStateVector;

	ProportionalNavigationTask(const ProportionalNavigationTask& CopySource); // Copy Constructor

	void runProportionalTaks();

	void calculatePositionVelocityRelatives(const SStateVector stateVector, SPositionVelocityRelatives& result);

	void lineOfSight(const SPositionVelocityRelatives& relative, SModules& modules, double(&lineOfSight)[3]);

	void velocityFeedbackPN(const SPositionVelocityRelatives relative, 
		const SModules modules, 
		const double lineOfSight[3], 
		SAcceleartion &acceleration);

	void plusControlPN(const SPositionVelocityRelatives& relative, 
		const SModules& modules, 
		const double(&lineOfSight)[3], 
		SAcceleartion &acceleration);

	void cinematics(const SStateVector stateVector, 
		const SAcceleartion& acceleration, 
		SDerivativeStateVector& derivative);

	void intermediateState(const SDerivativeStateVector& derivative, 
		const SStateVector& state, 
		SStateVector& stateTemp, 
		const double constant);

	void updateState(const SDerivativeStateVector derivativeS1,
		const SDerivativeStateVector derivativeS2,
		const SDerivativeStateVector derivativeS3,
		const SDerivativeStateVector derivativeS4,
		SStateVector& state);

	void process(SStateVector& stateVector,
		SStateVector& stateVectorTemp,
		SPositionVelocityRelatives& relative,
		SModules& modules,
		double(&lineOfSight)[3],
		SAcceleartion& acceleration,
		SDerivativeStateVector& derivative,
		const SDerivativeStateVector& lastDerivative,
		const bool isFirstDerivative = false,
		const double constant = 0.5);
};

} // namespace pfc

