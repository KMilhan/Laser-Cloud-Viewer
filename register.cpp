#include "commandline.hpp"
#include <sstream>

 int main (int ac, char* av[])
 { 
 	cout<<ac<<"Arguments received"<<endl;
   	float min; float max; float leaf_size; int iteration;
   if(ac!=5)
   	cout<<"Invalid arguments"<<endl;
   else
   {
   	stringstream min_str;
   	stringstream max_str;
   	stringstream leaf_size_str;
   	stringstream iteration_str;
   	min_str << av[1];
   	max_str << av[2];
   	leaf_size_str << av[3];
   	iteration_str << av[4];

    min_str >> min;
    max_str >> max;
    leaf_size_str >> leaf_size;
   	iteration_str >> iteration;

   	cout<<"min"<<min<<"max"<<max<<"leaf_size"<<leaf_size<<"iteration"<<iteration<<endl;
   }
 	


   model cloud_model;
   std::vector<string> file_path;

   for(int i = 0 ; ;i++)
   {
      string temp = "./temp/"+boost::lexical_cast<std::string>(i)+".pcd";
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
   cout<<"In Registration..."<<endl;
	

   
   if(ac==5)
   {
    cout<<cloud_model.registration(min, max, leaf_size, iteration);
   }
   else
   	cout<<cloud_model.registration();	


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
