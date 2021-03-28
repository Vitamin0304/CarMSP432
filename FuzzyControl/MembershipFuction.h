#ifndef FUZZYCONTROL_MEMBERSHIPFUCTION_H_
#define FUZZYCONTROL_MEMBERSHIPFUCTION_H_

namespace fuzzy
{
	class MembershipFuction
	{
	public:
		//����������
		virtual float getValue(float x) = 0;
		//������min����implied��Ļ���ֵ������Ϊ������
		virtual float Integral(float y) = 0;
		//������min����implied��Ĵ��Ա����Ļ���ֵ������Ϊ������
		virtual float IntegralWithX(float y) = 0;
	};
	//������������
	class TriangleMF : public MembershipFuction
	{
	public:
		TriangleMF(float a, float b, float c);
		~TriangleMF();
		float getValue(float x) override;
		float Integral(float y);
		float IntegralWithX(float y);
	private:
		float a;
		float b;
		float c;
	};
	//������������
	class TrapezoidMF : public MembershipFuction
	{
	public:
		TrapezoidMF(float a, float b, float c, float d);
		~TrapezoidMF();
		float getValue(float x) override;
		float Integral(float y);
		float IntegralWithX(float y);
	private:
		float a;
		float b;
		float c;
		float d;
	};
}

#endif /* FUZZYCONTROL_MEMBERSHIPFUCTION_H_ */
