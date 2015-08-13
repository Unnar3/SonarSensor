#include <math.h>

class dvlData{

public:
	struct positionEstimate{
		double logtime;
		double x;
		double y;
		double z;
		double yaw;
	};

private:
	struct dvld
	{
		double logtime;
		double vbx;
		double vby;
		double vbz;
		double bottomok;
		double vwx;
		double vwy;
		double vwz;
		double waterok;
		double yaw;
		double xt;
		double yt;
	};
	std::vector<dvld> dvlmeasurements;

public:
	void readLogFile(std::string log_file){

		std::cout << "Started reading DVL data!" << std::endl;

        std::ifstream file(log_file);
        std::string line; 

        int line_number = 0;
        std::cout.precision(13);
        double number = 0;
        dvld tmp_dvld;
        while (std::getline(file, line))
        {
            std::stringstream   linestream(line);
            std::string         data;

            if (line_number > 5){
                if (!line.empty() || line!=""){
                    // std::getline(linestream, data, '\t');  // read up-to the first tab (discard tab).
                    for(int i = 0; i < 32; i++){
                        linestream >> number;
                        if (i == 0){
                            tmp_dvld.logtime = number;
                        } else if (i == 7){
                            tmp_dvld.vwx = number;
                        } else if (i == 8){
                            tmp_dvld.vwy = number;
                        } else if (i == 9){
                            tmp_dvld.vwz = number;
                        } else if (i == 10){
                            tmp_dvld.waterok = number;
                        } else if (i == 11){
                            tmp_dvld.vbx = number;
                        } else if (i == 12){
                            tmp_dvld.vby = number;
                        } else if (i == 13){
                            tmp_dvld.vbz = number;
                        } else if (i == 14){
                            tmp_dvld.bottomok = number;
                        } else if (i == 22){
                        	tmp_dvld.yaw = number/360*2*M_PI;
                        } else if (i == 30){
                        	tmp_dvld.xt = number/360*2*M_PI;
                        } else if (i == 31){
                        	tmp_dvld.yt = number/360*2*M_PI;
                        }
                    }
                    dvlmeasurements.push_back(tmp_dvld);
                }
            }
            line_number++;
        }
        std::cout << "Finished reading DVL data!" << std::endl;
	};

	std::vector<positionEstimate> getPosition(){

		positionEstimate pose;
		std::vector<positionEstimate> vpose;

		double td = 0;
		double dx = 0;
		double dy = 0;
		dvld M;

		for (size_t i = 0; i < dvlmeasurements.size(); ++i){
			if ( dvlmeasurements.at(i).bottomok == 1 ){
				M = dvlmeasurements.at(i);
				if (i == 0){
					pose.logtime = M.logtime;
					pose.x = 0;
					pose.y = 0;
					pose.z = 0;
					pose.yaw = M.yaw;
				} else {
					td = M.logtime - dvlmeasurements.at(i-1).logtime;
					pose.logtime = M.logtime;
					pose.x = vpose.back().x + (M.vbx*td*cos(M.yaw) - M.vby*td*sin(M.yaw))/100;
					pose.y = vpose.back().y + (M.vbx*td*sin(M.yaw) + M.vby*td*cos(M.yaw))/100;
					pose.z = vpose.back().z;
					pose.yaw = M.yaw;
				}
				vpose.push_back(pose);
			}
		}
		return vpose;
	};


};