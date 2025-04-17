#include "PID.h"

PID_Solver::PID_Solver() //:
	//m_TargetValue(0.0f), m_CurrentValue(0.0f);
{
  
}

void PID_Solver::Init(FConstant InValues, bool bShouldEnableIntegral, bool bShouldEnableProportional, bool bShouldEnableDerivative)
{
	SetConstants(InValues);
	bIEnable = bShouldEnableIntegral;
	bPEnable = bShouldEnableProportional;
	bDEnable = bShouldEnableDerivative;
}

void PID_Solver::Tick()
{
	//Implement this in arduino IDE
}

static float PID_Solver::GetPIDValue(FResult* InValue)
{
	return InValue->Proportional + InValue->Integral + InValue->Derivative;
}

void PID_Solver::Solve()
{
	m_Error = m_TargetValue - m_CurrentValue;

	if (bPEnable)
	{
		m_ResultValues.Proportional = m_ConstantValues.Proportional * m_Error;
	}

	if (bIEnable)
	{
		m_ResultValues.Integral += m_ConstantValues.Integral * m_Error * m_DeltaTime;
	}

	if (bDEnable)
	{
		m_ResultValues.Derivative = (m_ConstantValues.Derivative * (m_Error - m_PastError)) / m_DeltaTime;
	}
  m_PastError = m_Error;
}

void PID_Solver::SetConstants(FConstant Values)
{
	m_ConstantValues.Integral = Values.Integral;
	m_ConstantValues.Derivative = Values.Integral;
	m_ConstantValues.Proportional = Values.Proportional;
}
