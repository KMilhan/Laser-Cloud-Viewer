#include "commandline.hpp"
#include <sstream>


 int main (int ac, char* av[])
 { 
 	cout<<ac<<"Arguments received"<<endl;

   //TODO: Implement argument receiver 
   
   if(ac<2)
   	cout<<"Invalid arguments"<<endl;
 	


   model cloud_model;
   std::vector<string> file_path;

   for(int i = 0 ; i < ac ;i++)
   {
      string temp = "./temp/MK"+boost::lexical_cast<std::string>(i)+".pcd";
      boost::filesystem::path p(temp);
      if(boost::filesystem::exists(p))
      {
      	file_path.push_back(temp);
     	cout<<file_path.back()<<endl; // find until last MK_i file then break.
	  }
      else
      	break;
      
   }
   cout<<"Opening..."<<endl;
   cout<<cloud_model.open_files(file_path)<<"Files"<<endl;

   string temp = string(av[1]);
   cout<<"Saving..."<<cloud_model.save_files(temp);


   //std::cout<<"Initilized"<<std::endl;
   //model cloud_model;
   //std::vector<std::string> file_path;
   //file_path.push_back("./data/P05S10.txt");
   //file_path.push_back("./data/P05S11.txt");
   //file_path.push_back("./data/P05S13.txt.pcd");
   //
   //std::cout<<cloud_model. open_files(file_path)<<std::endl;
   //std::cout<<cloud_model. registration()<<std::endl;
   //std::cout<<cloud_model. segmentation()<<std::endl;
   //cloud_model.visualize_cloud();   
   return 0;
 }
