#include "FuzzyController.h"
namespace fuzzy
{
	FuzzyControllerBase::FuzzyControllerBase(FuzzyVar** inputVars, FuzzyVar& outputVar)
		:outputVar(outputVar),inputVars(inputVars)
	{

	}
	FuzzyControllerBase::~FuzzyControllerBase()
	{
	}
	Fuzzy3InputsController::Fuzzy3InputsController(FuzzyVar** inputVars, FuzzyVar& outputVar, uint8_t* rules)
		:FuzzyControllerBase(inputVars, outputVar),rules(rules)
	{
		inputFuzzySetsNum[0] = inputVars[1]->getFuzzySetsNum();
		inputFuzzySetsNum[1] = inputVars[2]->getFuzzySetsNum();
	}

	uint8_t Fuzzy3InputsController::getRule(uint8_t i, uint8_t j, uint8_t k)
	{
		return *(rules + (inputVars[0]->fuzzySetsGrades[i].fuzzySetsIndex)
			* (inputFuzzySetsNum[0] * inputFuzzySetsNum[1])
			+ inputVars[1]->fuzzySetsGrades[j].fuzzySetsIndex * inputFuzzySetsNum[1]
			+ inputVars[2]->fuzzySetsGrades[k].fuzzySetsIndex);
	}

	float Fuzzy3InputsController::Compute(float* x)
	{
		//计算隶属度
		for (int i = 0; i < inputVarsNum; ++i)
		{
			inputVars[i]->ComputeFuzzySetsGrades(x[i]);
		}
		//重心法计算的分母和分子
		float integralSum = 0;
		float integralWithXSum = 0;
		//premise模糊前件
		float premise;

		for (int i = 0; i < inputVars[0]->fuzzySetsGradesNum; i++)
		{
			for (int j = 0; j < inputVars[1]->fuzzySetsGradesNum; j++)
			{
				for (int k = 0; k < inputVars[2]->fuzzySetsGradesNum; k++)
				{
					//定义乘法为模糊And运算
					premise = inputVars[0]->fuzzySetsGrades[i].grade
						* inputVars[1]->fuzzySetsGrades[j].grade
						* inputVars[2]->fuzzySetsGrades[k].grade;
					//COG重心法defuzzification解模糊
					integralSum += outputVar.getIntegral(getRule(i,j,k), premise);
					
					integralWithXSum += outputVar.getIntegralWithX(getRule(i, j, k), premise);
				}
			}
		}
		return integralWithXSum / integralSum;
	}
}
