#include "dgame.h"

void example3()
{
  typedef mvsystem_c< dubins_velocity_c, mvmap_c<4>, mvregion_c<4>, cost_c<4>, automaton_product_c<2> > system_t;
  typedef system_t::state state;
  typedef typename system_t::control control;
  typedef typename system_t::trajectory trajectory;
  typedef typename system_t::region_t region;

  lcm_t* lcm          = bot_lcm_get_global(NULL);
  bot_lcmgl_t* lcmgl  = bot_lcmgl_init(lcm, "plotter");
  bot_lcmgl_switch_buffer(lcmgl);

  double w = 12;
  double vmax = 1.5;
  double nomv = 0.5;
  double epsilon = 0.1;

  vector<pair<size_t, automaton_ss_c> > rules;
  rules.push_back( make_pair(0, automaton_ss_c(false, SIDEWALK, 1, 0)));
  rules.push_back( make_pair(1, automaton_ss_c(true, GOOD_DIR, 1, 0)));
  rules.push_back( make_pair(1, automaton_ss_c(false, LANE_CHANGE, 0, 10)));
  rules.push_back( make_pair(2, automaton_ss_c(false, SLOW, 1, 0)));
  
  double zero[4] = {0};
  double size[4] = {w, 6*w, 2*M_PI, 2*vmax};
  region op_region = region(zero, size);

  double sc1[4] = {w/4,-2*w, 0.5*M_PI, nomv};
  double gc1[4] = {w/4, 2*w, 0.5*M_PI, nomv};
  double gs[4] = {0.5, 0.5, 10*M_PI/180.0, 0.5};
  double gsplot[4] = {1, 1, 10*M_PI/180.0, 0.1};

  double sc2[4] = {-w/4, 1.5*w, -M_PI/2, nomv};
  double gc2[4] = {-w/4, -2*w, -M_PI/2, nomv};

  region gr1 = region(gc1, gs, red);
  region gr2 = region(gc2, gs, green);
  region sr1 = region(sc1, gs);
  region sr2 = region(sc2, gs);

  vector<region> regions, heuristic_sampling_regions;
  
  double de1[2] = {1, 0};
  double de2[2] = {0, 1};
  double de3[2] = {-1,0};
  double de4[2] = {0,-1};

  double rcr[4] = {w/4, 0, 0,0};
  double rcl[4] = {-w/4,0,0,0};
  double rsl[4] = {w/2,6*w,2*M_PI,2*vmax};

  double rcswr[4] = {5*w/8,0,0,0};
  double rcswl[4] = {-5*w/8,0,0,0};
  double rssw[4] = {w/4,6*w,2*M_PI,2*vmax};

  label_c right_lane(RIGHT_LANE);
  label_c left_lane(LEFT_LANE);
  label_c sidewalk(SIDEWALK);

  // rcrd = region,center, right-lane,down
  regions.push_back(region(rcr, rsl, de2, right_lane));
  regions.push_back(region(rcl, rsl, de4, left_lane));

  regions.push_back(region(rcswr, rssw, de2, sidewalk, dgrey));
  regions.push_back(region(rcswl, rssw, de4, sidewalk, dgrey));

  double hrcr[4] = {w/4,0,M_PI/2,nomv};
  double hrcl[4] = {-w/4,0,-M_PI/2,nomv};
  double hrs[4] = {w/2,6*w,M_PI/2,2*nomv};

  heuristic_sampling_regions.push_back(region(hrcr,hrs));
  heuristic_sampling_regions.push_back(region(hrcl,hrs));

  dgame_c<system_t> dgame(lcm, lcmgl);
  dgame.insert_rules(rules);
  dgame.initialize(op_region, sr1, gr1, sr2, gr2, regions);
  dgame.insert_heuristic_regions(heuristic_sampling_regions);

  vector<region> obstacles;
  double ocr[4] = {w/4,-w/2, 0, 0};
  double os[4] = {w/2,w, 2.0*M_PI, 2*vmax};

  obstacles.push_back(region(ocr, os, pink)); 
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
    
    double t1[3] = {0, -3*w, 0};
    double t2[3] = {0, 3*w, 0};
    draw_line(t1, t2);
  };

  auto draw_regions = [&](){

    bot_lcmgl_enable(lcmgl, GL_BLEND); 
    for(auto r : regions)
    {
      if( (r.label == sidewalk))
        dgame.p1.frrts.plot_region(r,2);
    }
    bot_lcmgl_disable(lcmgl, GL_BLEND); 
    
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

  for(auto i : range(0, 2000))
  {
    dgame.iteration();
    if(i% 100 == 0)
    {
      dgame.draw_tree(i);
      draw_all();
      bot_lcmgl_switch_buffer(lcmgl);
      
      cout<<"i: "<< i << endl;
      getchar();
    }
  }

  dgame.draw_tree(1000);
  draw_all();
  bot_lcmgl_switch_buffer(lcmgl);

  return;
}
