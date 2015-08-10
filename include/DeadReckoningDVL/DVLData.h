#include <math.h>

class dvlData{

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
                    }
                    dvlmeasurements.push_back(tmp_dvld);
                }
            }
            line_number++;
        }
        std::cout << "Finished reading DVL data!" << std::endl;
	}


};