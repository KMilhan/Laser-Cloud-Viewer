
#include "model.hpp"
#ifndef ui_h
#define ui_h
#include <FL/Fl.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/Fl_Spinner.H>
#include <FL/Fl_Text_Display.H>

class laserApp {
  public:
    model cloud_model;

  private:
    int g_meank;
    float g_stddev;
    float g_bottom;
    float g_top;
    float g_leaf_size;
    int g_max_icp;
    float g_voxel_leaf;
    int g_seg_max_iter;
    int g_dist;
    double g_cluter_tol;
    int g_min_clu;
    int g_max_clu;

  public:
    laserApp();
    Fl_Double_Window *basicWindow;
    Fl_Button *open_button;

  private:
    inline void cb_open_button_i(Fl_Button *, void *);
    static void cb_open_button(Fl_Button *, void *);
    inline void cb_Register_i(Fl_Button *, void *);
    static void cb_Register(Fl_Button *, void *);
    inline void cb_Noise_i(Fl_Button *, void *);
    static void cb_Noise(Fl_Button *, void *);
    inline void cb_Segment_i(Fl_Button *, void *);
    static void cb_Segment(Fl_Button *, void *);
    inline void cb_Save_i(Fl_Button *, void *);
    static void cb_Save(Fl_Button *, void *);
    inline void cb_Visualize_i(Fl_Button *, void *);
    static void cb_Visualize(Fl_Button *, void *);

  public:
    Fl_Text_Display *logWidget;
    Fl_Double_Window *advancedWindow;
    Fl_Spinner *meank;

  private:
    inline void cb_meank_i(Fl_Spinner *, void *);
    static void cb_meank(Fl_Spinner *, void *);

  public:
    Fl_Spinner *stddev;

  private:
    inline void cb_stddev_i(Fl_Spinner *, void *);
    static void cb_stddev(Fl_Spinner *, void *);

  public:
    Fl_Spinner *bottom;

  private:
    inline void cb_bottom_i(Fl_Spinner *, void *);
    static void cb_bottom(Fl_Spinner *, void *);

  public:
    Fl_Spinner *top;

  private:
    inline void cb_top_i(Fl_Spinner *, void *);
    static void cb_top(Fl_Spinner *, void *);

  public:
    Fl_Spinner *leaf_size;

  private:
    inline void cb_leaf_size_i(Fl_Spinner *, void *);
    static void cb_leaf_size(Fl_Spinner *, void *);

  public:
    Fl_Spinner *max_icp;

  private:
    inline void cb_max_icp_i(Fl_Spinner *, void *);
    static void cb_max_icp(Fl_Spinner *, void *);

  public:
    Fl_Spinner *voxel_leaf;

  private:
    inline void cb_voxel_leaf_i(Fl_Spinner *, void *);
    static void cb_voxel_leaf(Fl_Spinner *, void *);

  public:
    Fl_Spinner *seg_max_iter;

  private:
    inline void cb_seg_max_iter_i(Fl_Spinner *, void *);
    static void cb_seg_max_iter(Fl_Spinner *, void *);

  public:
    Fl_Spinner *dist;

  private:
    inline void cb_dist_i(Fl_Spinner *, void *);
    static void cb_dist(Fl_Spinner *, void *);

  public:
    Fl_Spinner *cluster_tol;

  private:
    inline void cb_cluster_tol_i(Fl_Spinner *, void *);
    static void cb_cluster_tol(Fl_Spinner *, void *);

  public:
    Fl_Spinner *min_clu;

  private:
    inline void cb_min_clu_i(Fl_Spinner *, void *);
    static void cb_min_clu(Fl_Spinner *, void *);

  public:
    Fl_Spinner *max_clu;

  private:
    inline void cb_max_clu_i(Fl_Spinner *, void *);
    static void cb_max_clu(Fl_Spinner *, void *);
    void log(const char *text);
};
#endif
