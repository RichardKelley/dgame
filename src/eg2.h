#include "dgame.h"

void example2()
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

  regions.push_back(region(rcrd, rsy, de2, right_lane, grey));
  regions.push_back(region(rcru, rsy, de2, right_lane, grey));
  regions.push_back(region(rcld, rsy, de4, left_lane));
  regions.push_back(region(rclu, rsy, de4, left_lane));

  regions.push_back(region(rcrr, rsx, de1, right_lane, grey));
  regions.push_back(region(rcrl, rsx, de1, right_lane, grey));
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
