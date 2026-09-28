// Project Headers
#include "ProportionalNavigationTask.h"


int main()
{
    using pfc::ProportionalNavigationTask;

    ProportionalNavigationTask::SStateVector init{};
    init.m_targetPositionX = 22.0;
    init.m_targetPositionY = 10.0;
    init.m_targetPositionZ = 30.0;
    init.m_interceptorPositionX = 0.0;
    init.m_interceptorPositionY = 0.0;
    init.m_interceptorPositionZ = 0.0;
    init.m_targetVelocityX = 5.0;
    init.m_targetVelocityY = 6.0;
    init.m_targetVelocityZ = 0.0;
    init.m_interceptorVelocityX = 0.0;
    init.m_interceptorVelocityY = 0.0;
    init.m_interceptorVelocityZ = 0.0;

    ProportionalNavigationTask task(init);
    // task.runProportionalTaks();

    task.saveToCSV_throughPointer("simulation_output.csv");
    task.saveToCSV_throughVector("simulation_output_alternative.csv");

    return 0;
}
