#include "commandline.hpp"
#include <sstream>


 int main (int ac, char* av[])
 { 
 	cout<<ac<<"Arguments received"<<endl;
   	int meank; float stddev;
   if(ac!=3)
   	cout<<"Invalid arguments"<<endl;
   else
   {
   	stringstream meank_str;
   	stringstream stddev_str;
   	meank_str << av[1];
   	stddev_str << av[2];
   	meank_str >> meank;
      stddev_str >> stddev;
   }
 	


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
   cout<<"In noise canceling..."<<endl;
   

   
   if(ac==3)
   {
    cout<<cloud_model.noise_cancel(meank, stddev);
   }
   else
   	cout<<cloud_model.noise_cancel();	

//UPDATE TEMP DIR
      for(int i = 0 ;  ;i++)
   {
      string temp = "./temp/"+boost::lexical_cast<std::string>(i)+".pcd";
      boost::filesystem::path p(temp);
      if(boost::filesystem::exists(p))
      {
        boost::filesystem::remove_all(p); // delete until last MK_i file then break.
         cout<<"Deleting... "<<temp<<endl;
     }
      else
         break;
      
   }
   cout<<"Saving..."<<cloud_model.save_files("./temp/");
//UPDATE TEMP DIR

   
   cloud_model.visualize_cloud();

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
