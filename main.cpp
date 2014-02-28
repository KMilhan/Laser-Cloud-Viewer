#include "ui.h"

int main (int argc, char** argv)
{
  //initialize class
laserApp app;
app.show;
  std::cout<<"Initilized"<<std::endl;
  
/*
  //std::cout<<cloud_model.registration()<<std::endl;
  //std::cout<<cloud_model.segmentation()<<std::endl;
  //cloud_model.visualize_cloud();

//  Fl_Window win( 300,500,"Testing" );
//  win.begin();
//  Fl_Button open_test( 10, 10, 180, 30, "Open Test file");
//  Fl_Button vis( 10, 110, 180, 30, "Visualize IT!" );
//  Fl_Button reg( 10, 210, 180, 30, "REGISTER!" );
//  Fl_Button seg( 10, 310, 180, 30, "Segment!!!" );
//  win.end();
//  reg.callback(reg_cb);
  vis.callback( but_cb );
  open_test.callback(test_cb);
  seg.callback(seg_cb);
  
  win.show();
  std::cout<<Fl::run()<<std::endl;
*/
  return 0;
}


