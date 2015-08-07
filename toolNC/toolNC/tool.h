
#define TOOL_H_
#define PI 3.1415926

class NCtool
{
public:
	static void toolcircle(double mea_pos, 
		                   double rev_spe, 
				           int frame,
					       const char* filename);
	static void toolcutlen(double helixAgl,
		                   double toolR, 
				           double cutEdgelen, 
					       int frame,
					       const char* filename);
	static void toolEdgecircle(double mea_pos,
		                       double toolR, 
				       		   int frame,
						       const char* filename);
};