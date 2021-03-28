#include "FuzzyVar.h"
namespace fuzzy
{
	FuzzyVar::FuzzyVar(MembershipFuction** fuzzySets, uint8_t fuzzySetsNum, float* range)
		:fuzzySets(fuzzySets), fuzzySetsNum(fuzzySetsNum), range(range)
	{
		
	}
	FuzzyVar::~FuzzyVar()
	{
	}
	//float FuzzyVar::operator()(uint8_t fuzzySetsIndex, float x)
	//{
	//	return fuzzySets[fuzzySetsIndex](x);
	//}
	void FuzzyVar::ComputeFuzzySetsGrades(float x)
	{
		FuzzySetsGrade fuzzySetsGrade;
		float grade;

		if (x < range[0])
			x = range[0];
		else if (x > range[1])
			x = range[1];

		fuzzySetsGradesNum = 0;
		for (int i = 0; i < fuzzySetsNum; ++i)
		{
			grade = fuzzySets[i]->getValue(x);
			if (grade > epsilon)
			{
				fuzzySetsGrade.fuzzySetsIndex = i;
				fuzzySetsGrade.grade = grade;
				fuzzySetsGrades[fuzzySetsGradesNum] = fuzzySetsGrade;
				fuzzySetsGradesNum++;
			}
		}
	}
}
