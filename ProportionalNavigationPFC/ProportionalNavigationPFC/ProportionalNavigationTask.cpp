// Project Headers
#include "ProportionalNavigationTask.h"
#include <fstream> 


// Definitions
pfc::ProportionalNavigationTask::ProportionalNavigationTask(SStateVector initialState) :
	m_initialState{initialState}, 
	m_processIntermediateStateVector{},
	m_processPoisitionVelocityRelatives{},
	m_processModules{},
	m_processLOS{},
	m_processAcceleration{},
	m_processDerivativeS1{},
	m_processDerivativeS2{},
	m_processDerivativeS3{},
	m_processDerivativeS4{},
	m_totalSteps{ static_cast<int>(s_FINAL_TIME / s_STEP_TIME) }
{
	m_vStateVector.clear();
	m_vStateVector.reserve(m_totalSteps);

	m_pStateVector = new SStateVector[m_totalSteps];

	runProportionalTaks();
}

// Copy Constructor 
// When an instance of a class is copied, a copy constructor is used to allocate memory dynamically at a different location to ensure that the destructor does not delete the same memory twice
pfc::ProportionalNavigationTask::ProportionalNavigationTask(const ProportionalNavigationTask& CopySource) :
	m_initialState{CopySource.m_initialState},
	m_processIntermediateStateVector{},
	m_processPoisitionVelocityRelatives{},
	m_processModules{},
	m_processLOS{},
	m_processAcceleration{},
	m_processDerivativeS1{},
	m_processDerivativeS2{},
	m_processDerivativeS3{},
	m_processDerivativeS4{},
	m_totalSteps{ static_cast<int>(s_FINAL_TIME / s_STEP_TIME) }
{
	m_vStateVector.clear();
	m_vStateVector.reserve(m_totalSteps);

	m_pStateVector = new SStateVector[m_totalSteps];

	runProportionalTaks();
}


pfc::ProportionalNavigationTask::~ProportionalNavigationTask()
{
	delete[] m_pStateVector;
}


void 
pfc::ProportionalNavigationTask::runProportionalTaks()
{
	const int s_STEPS = static_cast<int>(s_FINAL_TIME / s_STEP_TIME);

	SDerivativeStateVector dummyLast = {};

	SStateVector localStateVector = m_initialState;

	m_vStateVector.push_back(localStateVector);
	m_pStateVector[0] = localStateVector;

	for (int counter = 1; counter < s_STEPS; counter++)
	{
		process(localStateVector, m_processIntermediateStateVector, m_processPoisitionVelocityRelatives, m_processModules, m_processLOS, m_processAcceleration, m_processDerivativeS1, dummyLast, true);
		process(localStateVector, m_processIntermediateStateVector, m_processPoisitionVelocityRelatives, m_processModules, m_processLOS, m_processAcceleration, m_processDerivativeS2, m_processDerivativeS1);
		process(localStateVector, m_processIntermediateStateVector, m_processPoisitionVelocityRelatives, m_processModules, m_processLOS, m_processAcceleration, m_processDerivativeS3, m_processDerivativeS2);
		process(localStateVector, m_processIntermediateStateVector, m_processPoisitionVelocityRelatives, m_processModules, m_processLOS, m_processAcceleration, m_processDerivativeS4, m_processDerivativeS3, false, 1.0);

		updateState(m_processDerivativeS1, m_processDerivativeS2, m_processDerivativeS3, m_processDerivativeS4, localStateVector);

		m_vStateVector.push_back(localStateVector);
		m_pStateVector[counter] = localStateVector;
	}

}

void
pfc::ProportionalNavigationTask::process(
	SStateVector& stateVector,
	SStateVector& intermediateStateVector,
	SPositionVelocityRelatives& relative,
	SModules& modules,
	double(&LOS)[3],
	SAcceleartion& acceleration,
	SDerivativeStateVector& derivative,
	const SDerivativeStateVector& lastDerivative,
	const bool isFirstDerivative,
	const double constant
	)
{
	const SStateVector* currentStateVector = &stateVector;

	if (!isFirstDerivative)
	{
		intermediateState(lastDerivative, stateVector, intermediateStateVector, constant);
		currentStateVector = &intermediateStateVector;
	}


	calculatePositionVelocityRelatives(*currentStateVector, relative);
	lineOfSight(relative, modules, LOS);
	plusControlPN(relative, modules, LOS, acceleration);
	cinematics(*currentStateVector, acceleration, derivative);

}


void
pfc::ProportionalNavigationTask::updateState(
	const SDerivativeStateVector derivativeS1,
	const SDerivativeStateVector derivativeS2,
	const SDerivativeStateVector derivativeS3, 
	const SDerivativeStateVector derivativeS4, 
	SStateVector& state)
{
	state.m_targetPositionX += (s_STEP_TIME / 6.0) * (derivativeS1.m_DeriTargetPositionX + 2.0 * derivativeS2.m_DeriTargetPositionX +
		2.0 * derivativeS3.m_DeriTargetPositionX + derivativeS4.m_DeriTargetPositionX);
	state.m_targetPositionY += (s_STEP_TIME / 6.0) * (derivativeS1.m_DeriTargetPositionY + 2.0 * derivativeS2.m_DeriTargetPositionY +
		2.0 * derivativeS3.m_DeriTargetPositionY + derivativeS4.m_DeriTargetPositionY);
	state.m_targetPositionZ += (s_STEP_TIME / 6.0) * (derivativeS1.m_DeriTargetPositionZ + 2.0 * derivativeS2.m_DeriTargetPositionZ +
		2.0 * derivativeS3.m_DeriTargetPositionZ + derivativeS4.m_DeriTargetPositionZ);

	state.m_interceptorPositionX += (s_STEP_TIME / 6.0) * (derivativeS1.m_DeriInterceptorPositionX + 2.0 * derivativeS2.m_DeriInterceptorPositionX +
		2.0 * derivativeS3.m_DeriInterceptorPositionX + derivativeS4.m_DeriInterceptorPositionX);
	state.m_interceptorPositionY += (s_STEP_TIME / 6.0) * (derivativeS1.m_DeriInterceptorPositionY + 2.0 * derivativeS2.m_DeriInterceptorPositionY +
		2.0 * derivativeS3.m_DeriInterceptorPositionY + derivativeS4.m_DeriInterceptorPositionY);
	state.m_interceptorPositionZ += (s_STEP_TIME / 6.0) * (derivativeS1.m_DeriInterceptorPositionZ + 2.0 * derivativeS2.m_DeriInterceptorPositionZ +
		2.0 * derivativeS3.m_DeriInterceptorPositionZ + derivativeS4.m_DeriInterceptorPositionZ);

	state.m_targetVelocityX += (s_STEP_TIME / 6.0) * (derivativeS1.m_DeriTargetVelocityX + 2.0 * derivativeS2.m_DeriTargetVelocityX +
		2.0 * derivativeS3.m_DeriTargetVelocityX + derivativeS4.m_DeriTargetVelocityX);
	state.m_targetVelocityY += (s_STEP_TIME / 6.0) * (derivativeS1.m_DeriTargetVelocityY + 2.0 * derivativeS2.m_DeriTargetVelocityY +
		2.0 * derivativeS3.m_DeriTargetVelocityY + derivativeS4.m_DeriTargetVelocityY);
	state.m_targetVelocityZ += (s_STEP_TIME / 6.0) * (derivativeS1.m_DeriTargetVelocityZ + 2.0 * derivativeS2.m_DeriTargetVelocityZ +
		2.0 * derivativeS3.m_DeriTargetVelocityZ + derivativeS4.m_DeriTargetVelocityZ);

	state.m_interceptorVelocityX += (s_STEP_TIME / 6.0) * (derivativeS1.m_DeriInterceptorVelocityX + 2.0 * derivativeS2.m_DeriInterceptorVelocityX +
		2.0 * derivativeS3.m_DeriInterceptorVelocityX + derivativeS4.m_DeriInterceptorVelocityX);
	state.m_interceptorVelocityY += (s_STEP_TIME / 6.0) * (derivativeS1.m_DeriInterceptorVelocityY + 2.0 * derivativeS2.m_DeriInterceptorVelocityY +
		2.0 * derivativeS3.m_DeriInterceptorVelocityY + derivativeS4.m_DeriInterceptorVelocityY);
	state.m_interceptorVelocityZ += (s_STEP_TIME / 6.0) * (derivativeS1.m_DeriInterceptorVelocityZ + 2.0 * derivativeS2.m_DeriInterceptorVelocityZ +
		2.0 * derivativeS3.m_DeriInterceptorVelocityZ + derivativeS4.m_DeriInterceptorVelocityZ);
}


void
pfc::ProportionalNavigationTask::intermediateState(
	const SDerivativeStateVector& derivative,
	const SStateVector& state,
	SStateVector& stateTemp,
	const double constant)
{
	stateTemp.m_targetPositionX = state.m_targetPositionX + constant * s_STEP_TIME * derivative.m_DeriTargetPositionX;
	stateTemp.m_targetPositionY = state.m_targetPositionY + constant * s_STEP_TIME * derivative.m_DeriTargetPositionY;
	stateTemp.m_targetPositionZ = state.m_targetPositionZ + constant * s_STEP_TIME * derivative.m_DeriTargetPositionZ;

	stateTemp.m_interceptorPositionX = state.m_interceptorPositionX + constant * s_STEP_TIME * derivative.m_DeriInterceptorPositionX;
	stateTemp.m_interceptorPositionY = state.m_interceptorPositionY + constant * s_STEP_TIME * derivative.m_DeriInterceptorPositionY;
	stateTemp.m_interceptorPositionZ = state.m_interceptorPositionZ + constant * s_STEP_TIME * derivative.m_DeriInterceptorPositionZ;

	stateTemp.m_targetVelocityX = state.m_targetVelocityX + constant * s_STEP_TIME * derivative.m_DeriTargetVelocityX;
	stateTemp.m_targetVelocityY = state.m_targetVelocityY + constant * s_STEP_TIME * derivative.m_DeriTargetVelocityY;
	stateTemp.m_targetVelocityZ = state.m_targetVelocityZ + constant * s_STEP_TIME * derivative.m_DeriTargetVelocityZ;

	stateTemp.m_interceptorVelocityX = state.m_interceptorVelocityX + constant * s_STEP_TIME * derivative.m_DeriInterceptorVelocityX;
	stateTemp.m_interceptorVelocityY = state.m_interceptorVelocityY + constant * s_STEP_TIME * derivative.m_DeriInterceptorVelocityY;
	stateTemp.m_interceptorVelocityZ = state.m_interceptorVelocityZ + constant * s_STEP_TIME * derivative.m_DeriInterceptorVelocityZ;
}


void
pfc::ProportionalNavigationTask::cinematics(
	const SStateVector stateVector, 
	const SAcceleartion& acceleration,
	SDerivativeStateVector& derivative)
{
	derivative.m_DeriTargetPositionX = stateVector.m_targetVelocityX;
	derivative.m_DeriTargetPositionY = stateVector.m_targetVelocityY;
	derivative.m_DeriTargetPositionZ = stateVector.m_targetVelocityZ;

	derivative.m_DeriInterceptorPositionX = stateVector.m_interceptorVelocityX;
	derivative.m_DeriInterceptorPositionY = stateVector.m_interceptorVelocityY;
	derivative.m_DeriInterceptorPositionZ = stateVector.m_interceptorVelocityZ;

	/*
	derivative.m_DeriTargetVelocityX = acceleration.m_targetAccelerationX;
	derivative.m_DeriTargetVelocityY = acceleration.m_targetAccelerationY;
	derivative.m_DeriTargetVelocityZ = acceleration.m_targetAccelerationZ;
	*/

	derivative.m_DeriTargetVelocityX = s_ACCELERATION_TARGET_X;
	derivative.m_DeriTargetVelocityY = s_ACCELERATION_TARGET_Y;
	derivative.m_DeriTargetVelocityZ = s_ACCELERATION_TARGET_Z;

	derivative.m_DeriInterceptorVelocityX = acceleration.m_interceptorAccelerationX;
	derivative.m_DeriInterceptorVelocityY = acceleration.m_interceptorAccelerationY;
	derivative.m_DeriInterceptorVelocityZ = acceleration.m_interceptorAccelerationZ;
}


void 
pfc::ProportionalNavigationTask::velocityFeedbackPN(
	const SPositionVelocityRelatives relative,
	const SModules modules,
	const double lineOfSight[3],
	SAcceleartion &acceleration)
{
	// TBD
}


void
pfc::ProportionalNavigationTask::plusControlPN(
	const SPositionVelocityRelatives& relative,
	const SModules& modules,
	const double(&lineOfSight)[3],
	SAcceleartion &acceleration)
{
	double timeToGo = 0.0;

	if (modules.m_moduleVelocity < 1E-6)
	{
		timeToGo = modules.m_modulePosition / 1E-6;
	}
	else 
	{
		timeToGo = modules.m_modulePosition / modules.m_moduleVelocity;
	}
	
	double ZEM[STATE_DIM] = {};

	ZEM[X_COORDINATE] = relative.m_positionX + relative.m_velocityX * timeToGo;
	ZEM[Y_COORDINATE] = relative.m_positionY + relative.m_velocityY * timeToGo;
	ZEM[Z_COORDINATE] = relative.m_positionZ + relative.m_velocityZ * timeToGo;

	double termPlus[STATE_DIM] = {};

	termPlus[X_COORDINATE] = (1.0 / s_GAIN_H) * modules.m_modulePosition * modules.m_modulePosition * lineOfSight[X_COORDINATE];
	termPlus[Y_COORDINATE] = (1.0 / s_GAIN_H) * modules.m_modulePosition * modules.m_modulePosition * lineOfSight[Y_COORDINATE];
	termPlus[Z_COORDINATE] = (1.0 / s_GAIN_H) * modules.m_modulePosition * modules.m_modulePosition * lineOfSight[2];

	acceleration.m_interceptorAccelerationX = s_GAIN_N * ZEM[X_COORDINATE] / (timeToGo * timeToGo) + termPlus[X_COORDINATE];
	acceleration.m_interceptorAccelerationY = s_GAIN_N * ZEM[Y_COORDINATE] / (timeToGo * timeToGo) + termPlus[Y_COORDINATE];
	acceleration.m_interceptorAccelerationZ = s_GAIN_N * ZEM[Z_COORDINATE] / (timeToGo * timeToGo) + termPlus[Z_COORDINATE];
}


void
pfc::ProportionalNavigationTask::lineOfSight(
	const SPositionVelocityRelatives& relative, 
	SModules& modules,
	double (&lineOfSight)[3])
{

	modules.m_modulePosition = std::sqrt(relative.m_positionX * relative.m_positionX + relative.m_positionY * relative.m_positionY + relative.m_positionZ * relative.m_positionZ);
	modules.m_moduleVelocity = std::sqrt(relative.m_velocityX * relative.m_velocityX + relative.m_velocityY * relative.m_velocityY + relative.m_velocityZ * relative.m_velocityZ);

	lineOfSight[X_COORDINATE] = relative.m_positionX / modules.m_modulePosition;
	lineOfSight[Y_COORDINATE] = relative.m_positionY / modules.m_modulePosition;
	lineOfSight[Z_COORDINATE] = relative.m_positionZ / modules.m_modulePosition;

}


void 
pfc::ProportionalNavigationTask::calculatePositionVelocityRelatives(
	const SStateVector stateVector, 
	SPositionVelocityRelatives &result)
{
	result.m_positionX = stateVector.m_targetPositionX - stateVector.m_interceptorPositionX;
	result.m_positionY = stateVector.m_targetPositionY - stateVector.m_interceptorPositionY;
	result.m_positionZ = stateVector.m_targetPositionZ - stateVector.m_interceptorPositionZ;

	result.m_velocityX = stateVector.m_targetVelocityX - stateVector.m_interceptorVelocityX;
	result.m_velocityY = stateVector.m_targetVelocityY - stateVector.m_interceptorVelocityY;
	result.m_velocityZ = stateVector.m_targetVelocityZ - stateVector.m_interceptorVelocityZ;
}


void
pfc::ProportionalNavigationTask::saveToCSV_throughVector(const std::string& filename)
{
	std::ofstream file(filename);

	if (!file.is_open()) {
		std::cerr << "Error!\n";
		return;
	}

	file << "time,RTx,RTy,RTz,RIx,RIy,RIz,VTx,VTy,VTz,VIx,VIy,VIz\n";

	for (size_t i = 0; i < m_vStateVector.size(); ++i) {
		const auto& state = m_vStateVector[i];

		double t = i * s_STEP_TIME;

		file << t << ","
			<< state.m_targetPositionX << ","
			<< state.m_targetPositionY << ","
			<< state.m_targetPositionZ << ","
			<< state.m_interceptorPositionX << ","
			<< state.m_interceptorPositionY << ","
			<< state.m_interceptorPositionZ << ","
			<< state.m_targetVelocityX << ","
			<< state.m_targetVelocityY << ","
			<< state.m_targetVelocityZ << ","
			<< state.m_interceptorVelocityX << ","
			<< state.m_interceptorVelocityY << ","
			<< state.m_interceptorVelocityZ << "\n";
	}
}


void
pfc::ProportionalNavigationTask::saveToCSV_throughPointer(const char* filename)
{
	std::ofstream file(filename);

	if (!file.is_open()) {
		std::cerr << "Error!\n";
		return;
	}

	file << "time,RTx,RTy,RTz,RIx,RIy,RIz,VTx,VTy,VTz,VIx,VIy,VIz\n";

	for (int counter = 0; counter < m_totalSteps; counter++)
	{
		const SStateVector& state = m_pStateVector[counter]; // *(m_pStateVector + counter)

		double time = counter * s_STEP_TIME;

		file << time << ","
			<< state.m_targetPositionX << ","
			<< state.m_targetPositionY << ","
			<< state.m_targetPositionZ << ","
			<< state.m_interceptorPositionX << ","
			<< state.m_interceptorPositionY << ","
			<< state.m_interceptorPositionZ << ","
			<< state.m_targetVelocityX << ","
			<< state.m_targetVelocityY << ","
			<< state.m_targetVelocityZ << ","
			<< state.m_interceptorVelocityX << ","
			<< state.m_interceptorVelocityY << ","
			<< state.m_interceptorVelocityZ << "\n";
	}
}