#include "dgame.h"

void example1()
{
  typedef mvsystem_c<double_integrator_c, map_c<4>, mvregion_c<4>, cost_c<2>, automaton_product_c<0> > system_t;
  typedef system_t::state state;
  typedef typename system_t::control control;
  typedef typename system_t::trajectory trajectory;
  typedef typename system_t::region_t region;
  
  lcm_t *lcm          = bot_lcm_get_global(NULL);
  bot_lcmgl_t *lcmgl  = bot_lcmgl_init(lcm, "plotter");
  bot_lcmgl_switch_buffer(lcmgl);
  
  float w = 5;
  float maxv = 2;
  float epsilon = 0.2;

  vector<pair<size_t, automaton_ss_c> > rules;
  rules.push_back( make_pair(0, automaton_ss_c(true, GOOD_DIR, 1, 0)));
  rules.push_back( make_pair(1, automaton_ss_c(false, SLOW, 1, 0)));
 
  float zero[4] = {0};
  float size[4] = {3*w,3*w,2*maxv,2*maxv};
  region op_region = region(zero, size);
  
  float sc1[4] = {w/4,-w/2,0,1};
  float gc1[4] = {-w/2,w/4,-1,0};
  float gs[4] = {0.2,0.2,0.1,0.1};
  
  float sc2[4] = {-w/2,-w/4,1,0};
  float gc2[4] = {w/4,w/2,0,1};
  
  region gr1 = region(gc1, gs);
  region gr2 = region(gc2, gs);
  region sr1 = region(sc1, gs);
  region sr2 = region(sc2, gs);

  vector<region> regions;
  //rcrr = region_center_right-lane_right-hand-side
  float rcrd[4] = {w/4, -w, 0,0};
  float rcru[4] = {w/4, w, 0,0};
  float rcld[4] = {-w/4, -w, 0,0};
  float rclu[4] = {-w/4, w, 0,0};

  float rcrr[4] = {3*w/4, -w/4, 0,0};
  float rcrl[4] = {-3*w/4, w/4, 0,0};
  float rclr[4] = {3*w/4, w/4, 0,0};
  float rcll[4] = {-3*w/4, -w/4, 0,0};

  float rsx[4] = {w, w/2, 2*maxv, epsilon*maxv};
  float rsy[4] = {w/2, w, epsilon*maxv, 2*maxv};
  
  regions.push_back(region(rcrd, rsy));
  regions.push_back(region(rcru, rsy));
  regions.push_back(region(rcld, rsy));
  regions.push_back(region(rclu, rsy));
  
  regions.push_back(region(rcrr, rsx));
  regions.push_back(region(rcrl, rsx));
  regions.push_back(region(rclr, rsx));
  regions.push_back(region(rcll, rsx));

  dgame_c<system_t> dgame(lcmgl);
  dgame.insert_rules(rules);
  dgame.initialize(op_region, sr1, gr1, sr2, gr2, regions);
  
  for(auto i : range(0, 10000))
  {
    dgame.iteration();
    if(i%100 == 0)
    {
      dgame.draw(i);
      getchar();
    }
  }
  dgame.draw();

  return;
}
