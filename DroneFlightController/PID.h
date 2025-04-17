
#ifndef _PID_H_
#define _PID_H_

struct FConstant
{
	float Proportional{0.0f};
	float Integral{0.0f};
	float Derivative{0.0f};
};

typedef FConstant FResult;

class PID_Solver
{
public:

  PID_Solver();

	//void Init(FConstant InValues);

	void Init(FConstant InValues, bool bShouldEnableIntegral = true, bool bShouldEnableProportional = true, bool bShouldEnableDerivative = true);

	void Solve();

	void Tick();

  static float GetPIDValue(FResult* InValue);

private:

	float m_TargetValue{0.0f};
	float m_CurrentValue{0.0f};
	float m_Error{0.0f};
	float m_PastError{0.0f};

	float m_DeltaTime{0.0f};

	int m_ControlSignal;

	bool bIEnable{ false };
	bool bPEnable{ false };
	bool bDEnable{ false };

	FConstant m_ConstantValues;
	FResult m_ResultValues;

public:
	void SetConstants(FConstant Values);

	inline float GetTargetValue() const {return m_TargetValue;}
	inline void SetTargetValue(float NewValue) {m_TargetValue = NewValue;}

	inline float GetCurrentValue() const {return m_CurrentValue;}
	inline void SetCurrentValue(float NewValue) {m_CurrentValue = NewValue;}

	inline int GetControlSignal() const { return m_ControlSignal; }
	void SetControlSignal(int Invalue) { m_ControlSignal = Invalue; }

	inline FResult* GetResult() { return &m_ResultValues; }

	inline void SetDeltaTime(float InDeltaTime) { m_DeltaTime = InDeltaTime; }
};


#endif
