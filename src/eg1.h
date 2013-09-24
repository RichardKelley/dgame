#include "dgame.h"

template<size_t N>
class mvmap_c : public map_c<N>
{
  public:
    vector< mvregion_c<N> > obstacles;

    mvmap_c() : map_c<N> () {}
    void insert_obstacles(vector< mvregion_c<N> >& obstacles_in)
    {
      obstacles = obstacles_in;
    }
    bool is_in_collision(const float s[N])
    {
      for(auto& r : obstacles)
      {
        if(r.is_inside(s))
        {
          //cout<<"returning true"<<endl;
          return true;
        }
      }
      return false;
    }
};

double red[4] = {1,0,0,0.2};
double pink[4] = {0.8,0.3,0.3,0.2};
double grey[4] = {0,0,0,0.1};
double dgrey[4] = {0,0,0,0.3};
double ddgrey[4] = {0,0,0,0.8};
double yellow[4] = {1,1,0,0.5};
double green[4] = {0,1,0,0.2};

void example1()
{
  typedef mvsystem_c<single_integrator_c<2>, mvmap_c<2>, mvregion_c<2>, cost_c<4>, automaton_product_c<2> > system_t;
  typedef system_t::state state;
  typedef typename system_t::control control;
  typedef typename system_t::trajectory trajectory;
  typedef typename system_t::region_t region;

  lcm_t* lcm          = bot_lcm_get_global(NULL);
  bot_lcmgl_t* lcmgl  = bot_lcmgl_init(lcm, "plotter");
  bot_lcmgl_switch_buffer(lcmgl);

  double w = 2;
  double epsilon = 0.1;

  vector<pair<size_t, automaton_ss_c> > rules;
  rules.push_back( make_pair(0, automaton_ss_c(false, SIDEWALK, 1, 0)));
  rules.push_back( make_pair(1, automaton_ss_c(true, GOOD_DIR, 1, 0)));
  rules.push_back( make_pair(2, automaton_ss_c(false, LANE_CHANGE, 0, 1)));
  
  double zero[2] = {0};
  double size[2] = {3*w,3*w};
  region op_region = region(zero, size);

  double sc1[2] = {w/4,-3*w/2};
  double gc1[2] = {-3*w/2,w/4};
  double gs[2] = {0.1,0.1};

  double sc2[2] = {-3*w/2,-w/4};
  double gc2[2] = {w/4,3*w/2};

  region gr1 = region(gc1, gs, red);
  region gr2 = region(gc2, gs, green);
  region sr1 = region(sc1, gs);
  region sr2 = region(sc2, gs);

  vector<region> regions;
  
  double de1[2] = {1, 0};
  double de2[2] = {0, 1};
  double de3[2] = {-1,0};
  double de4[2] = {0,-1};

  double rcrd[2] = {w/4, -w};
  double rcru[2] = {w/4, w};
  double rcld[2] = {-w/4, -w};
  double rclu[2] = {-w/4, w};

  double rcswrd[2] = {w,-w};
  double rcswru[2] = {w,-w};
  double rcswld[2] = {-w,-w};
  double rcswlu[2] = {-w,-w};

  double rcrr[2] = {w, -w/4};
  double rcrl[2] = {-w, w/4};
  double rclr[2] = {w, w/4};
  double rcll[2] = {-w, -w/4};

  double rsx[2] = {w, w/2};
  double rsy[2] = {w/2, w};

  double rssw[2] = {w,w};

  label_c right_lane(RIGHT_LANE);
  label_c left_lane(LEFT_LANE);
  label_c sidewalk(SIDEWALK);

  regions.push_back(region(rcrd, rsy, de2, right_lane));
  regions.push_back(region(rcru, rsy, de2, right_lane));
  regions.push_back(region(rcld, rsy, de4, left_lane));
  regions.push_back(region(rclu, rsy, de4, left_lane));

  regions.push_back(region(rcrr, rsx, de1, right_lane));
  regions.push_back(region(rcrl, rsx, de1, right_lane));
  regions.push_back(region(rclr, rsx, de3, left_lane));
  regions.push_back(region(rcll, rsx, de3, left_lane));

  dgame_c<system_t> dgame(lcm, lcmgl);
  dgame.insert_rules(rules);
  dgame.initialize(op_region, sr1, gr1, sr2, gr2, regions);
  
  vector<region> obstacles;
  double ocrd[2] = {w,-w};
  double ocru[2] = {w,w};
  double oclu[2] = {-w, w};
  double ocld[2] = {-w, -w};
  double os[2] = {w,w};

  obstacles.push_back(region(ocrd, os, ddgrey)); 
  obstacles.push_back(region(ocru, os, ddgrey)); 
  obstacles.push_back(region(ocld, os, ddgrey)); 
  obstacles.push_back(region(oclu, os, ddgrey)); 

  dgame.insert_obstacles(obstacles);

  for(auto i : range(0, 10000))
  {
    dgame.iteration();
    if(i%100 == 0)
    {
      cout<<i<<endl;
      dgame.draw_tree(i);
      //getchar();
    }
  }
  dgame.draw_tree();

  return;
}


void example2()
{
  typedef mvsystem_c< dubins_velocity_c, mvmap_c<4>, mvregion_c<4>, cost_c<3>, automaton_product_c<1> > system_t;
  typedef system_t::state state;
  typedef typename system_t::control control;
  typedef typename system_t::trajectory trajectory;
  typedef typename system_t::region_t region;

  lcm_t* lcm          = bot_lcm_get_global(NULL);
  bot_lcmgl_t* lcmgl  = bot_lcmgl_init(lcm, "plotter");
  bot_lcmgl_switch_buffer(lcmgl);

  double w = 6;
  double vmax = 1.5;
  double nomv = 0.5;
  double epsilon = 0.1;

  vector<pair<size_t, automaton_ss_c> > rules;
  rules.push_back( make_pair(0, automaton_ss_c(false, SIDEWALK, 1, 0)));
  rules.push_back( make_pair(1, automaton_ss_c(true, GOOD_DIR, 1, 0)));
  rules.push_back( make_pair(1, automaton_ss_c(false, LANE_CHANGE, 0, 10)));
  //rules.push_back( make_pair(2, automaton_ss_c(false, SLOW, 1, 0)));
  
  double zero[4] = {0};
  double size[4] = {5*w,5*w, 2*M_PI, 2*vmax};
  region op_region = region(zero, size);

  double sc1[4] = {w/4,-5*w/2, 0.5*M_PI, nomv};
  double gc1[4] = {-5*w/2,w/4, -M_PI, nomv};
  double gs[4] = {0.1, 0.1, 10*M_PI/180.0, 0.1};
  double gsplot[4] = {1, 1, 10*M_PI/180.0, 0.1};

  double sc2[4] = {-5*w/2,-w/4, 0, nomv};
  double gc2[4] = {w/4,5*w/2, M_PI/2.0, nomv};

  region gr1 = region(gc1, gs, red);
  region gr2 = region(gc2, gs, green);
  region sr1 = region(sc1, gs);
  region sr2 = region(sc2, gs);

  vector<region> regions;
  
  double de1[2] = {1, 0};
  double de2[2] = {0, 1};
  double de3[2] = {-1,0};
  double de4[2] = {0,-1};

  double rcrd[4] = {w/4, -3*w/2, 0, 0};
  double rcru[4] = {w/4, 3*w/2, 0, 0};
  double rcld[4] = {-w/4, -3*w/2, 0, 0};
  double rclu[4] = {-w/4, 3*w/2, 0, 0};

  double rcswrd[4] = {3*w/2,-3*w/2, 0, 0};
  double rcswru[4] = {3*w/2,3*w/2, 0, 0};
  double rcswld[4] = {-3*w/2,3*w/2, 0, 0};
  double rcswlu[4] = {-3*w/2,-3*w/2, 0, 0};

  double rcrr[4] = {3*w/2, -w/4, 0, 0};
  double rcrl[4] = {-3*w/2, w/4, 0, 0};
  double rclr[4] = {3*w/2, w/4, 0, 0};
  double rcll[4] = {-3*w/2, -w/4, 0, 0};

  double rsx[4] = {w, w/2, 2.0*M_PI, 2*vmax};
  double rsy[4] = {w/2, w, 2.0*M_PI, 2*vmax};

  double rssw[4] = {2*w,2*w, 2.0*M_PI, 2*vmax};

  label_c right_lane(RIGHT_LANE);
  label_c left_lane(LEFT_LANE);
  label_c sidewalk(SIDEWALK);

  regions.push_back(region(rcrd, rsy, de2, right_lane));
  regions.push_back(region(rcru, rsy, de2, right_lane));
  regions.push_back(region(rcld, rsy, de4, left_lane));
  regions.push_back(region(rclu, rsy, de4, left_lane));

  regions.push_back(region(rcrr, rsx, de1, right_lane));
  regions.push_back(region(rcrl, rsx, de3, right_lane));
  regions.push_back(region(rclr, rsx, de3, left_lane));
  regions.push_back(region(rcll, rsx, de1, left_lane));

  regions.push_back(region(rcswrd, rssw, de1, sidewalk, grey));
  regions.push_back(region(rcswru, rssw, de1, sidewalk, grey));
  regions.push_back(region(rcswld, rssw, de1, sidewalk, grey));
  regions.push_back(region(rcswlu, rssw, de1, sidewalk, grey));

  dgame_c<system_t> dgame(lcm, lcmgl);
  dgame.insert_rules(rules);
  dgame.initialize(op_region, sr1, gr1, sr2, gr2, regions);
  
  vector<region> obstacles;
  double ocrd[4] = {13*w/8.0,-13*w/8.0, 0, 0};
  double ocru[4] = {13*w/8.0,13*w/8.0, 0, 0};
  double oclu[4] = {-13*w/8.0,13*w/8.0, 0, 0};
  double ocld[4] = {-13*w/8.0,-13*w/8.0, 0, 0};
  double os[4] = {7*w/4,7*w/4, 2.0*M_PI, 2*vmax};

  obstacles.push_back(region(ocrd, os, pink)); 
  obstacles.push_back(region(ocru, os, pink)); 
  obstacles.push_back(region(ocld, os, pink)); 
  obstacles.push_back(region(oclu, os, pink)); 

  dgame.insert_obstacles(obstacles);

  auto draw_line = [&](double* t1, double* t2){
    bot_lcmgl_begin(lcmgl, GL_LINES); 
    bot_lcmgl_vertex3f(lcmgl, t1[0], t1[1], t1[2]);
    bot_lcmgl_vertex3f(lcmgl, t2[0], t2[1], t2[2]);
    bot_lcmgl_end(lcmgl); 
  };

  auto draw_lines = [&](){
    bot_lcmgl_color4f(lcmgl, yellow[0], yellow[1], yellow[2], yellow[3]);
    bot_lcmgl_line_width(lcmgl, 4);
    
    double t1[3] = {0, -5*w/2, 0};
    double t2[3] = {0, -w/2, 0};
    draw_line(t1, t2);
    
    double t3[3] = {0, 5*w/2, 0};
    double t4[3] = {0, w/2, 0};
    draw_line(t3, t4);

    double t5[3] = {5*w/2, 0, 0};
    double t6[3] = {w/2, 0, 0};
    draw_line(t5, t6);

    double t7[3] = {-5*w/2, 0, 0};
    double t8[3] = {-w/2, 0, 0};
    draw_line(t7, t8);
  };

  auto draw_regions = [&](){

    for(auto r : regions)
    {
      if(r.label == sidewalk) 
        dgame.p1.frrts.plot_region(r,2);
    }
    region gr1p = region(gc1, gsplot, red);
    region gr2p = region(gc2, gsplot, green);
    bot_lcmgl_enable(lcmgl, GL_BLEND); 
    dgame.p1.frrts.plot_region(gr1p,2);
    dgame.p1.frrts.plot_region(gr2p,2);
    bot_lcmgl_disable(lcmgl, GL_BLEND); 

    for(auto o : obstacles)
      dgame.p1.frrts.plot_region(o,2);
  };
 
  auto draw_all = [&](){
    draw_lines();
    draw_regions();
  };

  for(auto i : range(0, 5000))
  {
    dgame.iteration();
    if(i%500 == 0)
    {
      cout<<i<<endl;
      dgame.draw_tree(i);
      draw_all();
      bot_lcmgl_switch_buffer(lcmgl);
    }
  }
  dgame.draw_tree(1000);
  draw_all();
  bot_lcmgl_switch_buffer(lcmgl);

  return;
}
