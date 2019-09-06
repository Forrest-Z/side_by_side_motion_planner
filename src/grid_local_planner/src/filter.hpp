#ifndef __FILTER_H__
#define __FILTER_H__

enum type_t {
	FILTER_HPF,
	FILTER_LPF
};

class filter
{
private:
	enum type_t type;
	double time_const;
	double x;
	double k[4];
public:
	filter( enum type_t type, double tc, double out0 )
	{
		time_const = tc;
		switch( type )
		{
		case FILTER_LPF:
			k[3] = - 1 / ( 1.0 + 2 * time_const );
			k[2] = - k[3];
			k[1] = ( 1.0 - 2 * time_const ) * k[3];
			k[0] = - k[1] - 1.0;
			x = ( 1 - k[2] ) * out0 / k[3];
			break;
		case FILTER_HPF:
			k[3] = - 1 / ( 1.0 + 2 * time_const );
			k[2] = - k[3] * 2 * time_const ;
			k[1] = ( 1.0 - 2 * time_const ) * k[3];
			k[0] = 2 * time_const * ( - k[1] + 1.0 );
			x = ( 1 - k[2] ) * out0 / k[3];
			break;
		}
	}
	double in( double i )
	{
		x = k[0] * i + k[1] * x;
		return k[2] * i + k[3] * x;
	}
};

#endif

