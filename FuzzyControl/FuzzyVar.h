#ifndef FUZZYCONTROL_FUZZYVAR_H_
#define FUZZYCONTROL_FUZZYVAR_H_

#include "MembershipFuction.h"
#include <stdint.h>

namespace fuzzy
{
	const float epsilon = 0.0001f;

	typedef struct Fuzzy_Sets_Grade
	{
		uint8_t fuzzySetsIndex;//模糊集序号
		float grade;//隶属度
		Fuzzy_Sets_Grade() : fuzzySetsIndex(0), grade(0) {}
	}FuzzySetsGrade;

	class FuzzyVar
	{
	public:
		FuzzyVar(MembershipFuction** fuzzySets, uint8_t fuzzySetsNum, float* range);
		~FuzzyVar();
		//计算x在这个模糊集的隶属度
		//float operator()(uint8_t fuzzySetsIndex, float x);

		//计算x在每个模糊集的隶属度，并将非零项存储
		void ComputeFuzzySetsGrades(float x);
		//隶属度不为0的模糊集及其隶属度列表
		FuzzySetsGrade fuzzySetsGrades[10];
		//隶属度不为0的模糊集的数量
		uint8_t fuzzySetsGradesNum = 0;

		//计算指定模糊集implied后的积分
		float getIntegral(uint8_t fuzzySetsIndex, float y)
		{
			return fuzzySets[fuzzySetsIndex]->Integral(y);
		}
		//计算指定模糊集implied后的带x的积分
		float getIntegralWithX(uint8_t fuzzySetsIndex, float y)
		{
			return fuzzySets[fuzzySetsIndex]->IntegralWithX(y);
		}

		uint8_t getFuzzySetsNum() { return fuzzySetsNum; }
	private:
		//模糊集的数量
		uint8_t fuzzySetsNum;
		//模糊集，用隶属函数表示
		MembershipFuction** fuzzySets;
		//模糊变量的论域 数组长度为2
		float* range;
	};
}

#endif /* FUZZYCONTROL_FUZZYVAR_H_ */
