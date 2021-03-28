#ifndef FUZZYCONTROL_FUZZYCONTROLLER_H_
#define FUZZYCONTROL_FUZZYCONTROLLER_H_

#include "FuzzyVar.h"
namespace fuzzy
{
	class FuzzyControllerBase
	{
	public:
		FuzzyControllerBase(FuzzyVar** inputVars, FuzzyVar& outputVar);
		virtual ~FuzzyControllerBase();

		virtual float Compute(float* x) = 0;

	protected:
		FuzzyVar** inputVars;
		FuzzyVar outputVar;
	};

	class Fuzzy3InputsController :public FuzzyControllerBase
	{
	public:
		Fuzzy3InputsController(FuzzyVar** inputVars, FuzzyVar& outputVar, uint8_t* rules);
		float Compute(float* x);
		uint8_t getRule(uint8_t i, uint8_t j, uint8_t k);
	private:
		const uint8_t inputVarsNum = 3;
		uint8_t* rules;
		uint8_t inputFuzzySetsNum[2];
	};
}

#endif /* FUZZYCONTROL_FUZZYCONTROLLER_H_ */
