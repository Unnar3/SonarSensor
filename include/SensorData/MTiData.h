#include "utils/utils.h"
class MTiData{
public:
	struct mtid{
		double logtime;
		double Roll;
		double Pitch;
		double Yaw;
		double rX;
		double rY;
		double rZ;
		double aX;
		double aY;
		double aZ;
	};
private:
	std::vector<mtid> vmtid;

public:
	std::vector<mtid> readMTiLOG(std::string log_file){
		std::cout << "Started reading DVL data!" << std::endl;

	        std::ifstream file(log_file);
	        std::string line; 
	
	        int line_number = 0;
	        std::cout.precision(13);
	        double number = 0;
	        mtid tmp_mtid;
	
	        while (std::getline(file, line))
	        {
	            std::stringstream   linestream(line);
	            std::string         data;
	
	            if (line_number > 6){
	                if (!line.empty() || line!=""){
	                    // std::getline(linestream, data, '\t');  // read up-to the first tab (discard tab).
	                    for(int i = 0; i < 10; i++){
	                        linestream >> number;
	                        if (i == 0){
	                        	tmp_mtid.logtime = number;
	                        } else if (i == 1) {
	                        	tmp_mtid.Roll = number;
	                        } else if (i == 2) {
	                        	tmp_mtid.Pitch = number;
	                        } else if (i == 3){
	         					tmp_mtid.Yaw = utils::deg2rad(number);                        	
	                        } else if (i == 4){
	                        	tmp_mtid.rX = number;
	                        } else if (i == 5){
	                        	tmp_mtid.rY = number;
	                        } else if (i == 6){
	                        	tmp_mtid.rZ = number;
	                        } else if (i == 7){
	                        	tmp_mtid.aX = number;
	                        } else if (i == 8){
	                        	tmp_mtid.aY = number;
	                        } else if (i == 9){
	                        	tmp_mtid.aZ = number;
	                        }
	                    }
	                    vmtid.push_back(tmp_mtid);
	                }
	            }
	            line_number++;
	        }
	        std::cout << "Finished reading MTi data!" << std::endl;
	        return vmtid;
	}

};
