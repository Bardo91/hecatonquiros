// This file find the continuos joints on a text file. We will use for BackendGazebo.cpp
// Luis Marzo Román

#include <iostream>
#include <iomanip>
#include <fstream>


int main() {

    
    int len_fixed_string = 0;
    int next_iteration=0;
	int i;
    int cnt_vector=0;
    std::ifstream inFile;
    std::string string;
    std::string var_string;
    std::string item_name;
    std::ifstream nameFileout;
    std::string string_vector[30];
    


nameFileout.open("/home/luis/catkin_ws_2/src/hecatonquiros/arm_description/urdf/4dof.urdf");
while (nameFileout >> item_name)
{
    //std::cout << item_name << "\n";
	
	if(next_iteration==2)
		{
			len_fixed_string=item_name.length();
			if(len_fixed_string==13)   //13 is the amount of characters in type="fixed">
			{
			//Deleting
			string_vector[cnt_vector]="deleted";
			}
			else
			{
			//Saving because the joint is continuous
			cnt_vector++;
			}

		next_iteration=0;
		}


	if(next_iteration==1)
		{
		string=item_name;	
		var_string.resize(string.length()-7); //7 is the number of characters that I dont need in the xml
		for(i=0;i<=(string.length()-7) ;i++)
		{
		var_string[i]=string[i+6];
		}
		string_vector[cnt_vector]=var_string;
		next_iteration++;
		}
	
	
	if(item_name=="<joint") //Search the joints
		{
			next_iteration++;
		}
	

}
for(i=0;i<cnt_vector;i++)
{
std::cout << "\n" << string_vector[i];
}
nameFileout.close();
    return 0;
}
