#include "dgame.h"

int test_rrt()
{
  typedef mvsystem_c<dubins_c, map_c<3>, mvregion_c<3>, cost_c<4>, automaton_product_c<2> > system_t;
  typedef system_t::state state;
  typedef typename system_t::control control;
  typedef typename system_t::trajectory trajectory;
  typedef typename system_t::region_t region;
 
  lcm_t *lcm          = bot_lcm_get_global(NULL);
  bot_lcmgl_t *lcmgl  = bot_lcmgl_init(lcm, "plotter");
  bot_lcmgl_switch_buffer(lcmgl);
  
  vector<automaton_ss_c> rules;
  rules.push_back(automaton_ss_c(false, SIDEWALK, 1, 0));

  rrts_c<vertex_c<system_t>, edge_c<system_t> > rrts(lcmgl);

  float zero[3] = {0};
  float size[3] = {100,100,2*M_PI};
  rrts.system.operating_region = region(zero, size);

  float gc[3] = {10,10,0};
  float gs[3] = {1,1,0.1*M_PI};
  state goal_state(gc);
  rrts.system.goal_region = region(gc,gs);
  
  state origin(zero);
  rrts.initialize(origin);

  time_t ts=time(0), te;
  int max_iterations = 1e4, diter=max_iterations/10;
  for(int i=0; i<max_iterations; i++)
  {
    rrts.iteration();
    if(i%diter == 0){
      cout<<i<<" ";
      rrts.get_best_cost().print(cout, "(", ")\n");
    }
  }
  rrts.get_best_cost().print(cout, "(", ")\n");
  /*
  trajectory best_traj;
  rrts.get_best_trajectory(best_traj);
  best_traj.print();
  */
  cout<<"time: "<< difftime(time(0), ts)<<endl;

  return 0;
}


int main()
{
  srand(time(NULL));

  typedef mvsystem_c<double_integrator_c, map_c<4>, mvregion_c<4>, cost_c<2>, automaton_product_c<0> > system_t;
  typedef system_t::state state;
  typedef typename system_t::control control;
  typedef typename system_t::trajectory trajectory;
  typedef typename system_t::region_t region;
  
  lcm_t *lcm          = bot_lcm_get_global(NULL);
  bot_lcmgl_t *lcmgl  = bot_lcmgl_init(lcm, "plotter");
  bot_lcmgl_switch_buffer(lcmgl);
  
  vector<pair<size_t, automaton_ss_c> > rules;
  rules.push_back( make_pair(0, automaton_ss_c(false, SIDEWALK, 1, 0)));
 
  float zero[4] = {0};
  float size[4] = {25,25,4,4};

  float sc1[4] = {10,-10,0,0};
  float gc1[4] = {-10,10,0,1};
  float gs1[4] = {1,1,0.1,0.1};
  
  float sc2[4] = {-10,-10,0,0};
  float gc2[4] = {10,10,1,0};
  float gs2[4] = {1,1,0.1,0.1};
  
  region op_region = region(zero, size);
  region gr1 = region(gc1, gs1);
  region gr2 = region(gc2, gs2);
  region sr1 = region(sc1, gs1);
  region sr2 = region(sc2, gs2);

  vector<region> regions;
  
  dgame_c<system_t> dgame(lcmgl);
  dgame.insert_rules(rules);
  dgame.initialize(op_region, sr1, gr1, sr2, gr2, regions);
  
  for(auto i : range(0, 10000))
  {
    dgame.iteration();
    if(i%100 == 0)
      dgame.draw(i);
  }
  dgame.draw();

  return 0;
}
