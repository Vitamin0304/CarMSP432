#ifndef FUZZYCONTROL_FUZZYVAR_H_
#define FUZZYCONTROL_FUZZYVAR_H_

#include "MembershipFuction.h"
#include <stdint.h>

namespace fuzzy
{
	const float epsilon = 0.0001f;

	typedef struct Fuzzy_Sets_Grade
	{
		uint8_t fuzzySetsIndex;//ģ�������
		float grade;//������
		Fuzzy_Sets_Grade() : fuzzySetsIndex(0), grade(0) {}
	}FuzzySetsGrade;

	class FuzzyVar
	{
	public:
		FuzzyVar(MembershipFuction** fuzzySets, uint8_t fuzzySetsNum, float* range);
		~FuzzyVar();
		//����x�����ģ������������
		//float operator()(uint8_t fuzzySetsIndex, float x);

		//����x��ÿ��ģ�����������ȣ�����������洢
		void ComputeFuzzySetsGrades(float x);
		//�����Ȳ�Ϊ0��ģ���������������б�
		FuzzySetsGrade fuzzySetsGrades[10];
		//�����Ȳ�Ϊ0��ģ����������
		uint8_t fuzzySetsGradesNum = 0;

		//����ָ��ģ����implied��Ļ���
		float getIntegral(uint8_t fuzzySetsIndex, float y)
		{
			return fuzzySets[fuzzySetsIndex]->Integral(y);
		}
		//����ָ��ģ����implied��Ĵ�x�Ļ���
		float getIntegralWithX(uint8_t fuzzySetsIndex, float y)
		{
			return fuzzySets[fuzzySetsIndex]->IntegralWithX(y);
		}

		uint8_t getFuzzySetsNum() { return fuzzySetsNum; }
	private:
		//ģ����������
		uint8_t fuzzySetsNum;
		//ģ������������������ʾ
		MembershipFuction** fuzzySets;
		//ģ������������ ���鳤��Ϊ2
		float* range;
	};
}

#endif /* FUZZYCONTROL_FUZZYVAR_H_ */
