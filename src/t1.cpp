#include "mvsystem.h"

int main()
{
  typedef mvsystem_c<dubins_c, map_c<3>, cost_c<4>, automaton_product_c<2> > system_t;
  typedef system_t::state state;
  typedef typename system_t::control control;
  typedef typename system_t::trajectory trajectory;
  typedef typename system_t::region_t region;
 
  vector<automaton_ss_c> rules;
  rules.push_back(automaton_ss_c(false, SIDEWALK, 1, 0));

  rrts_c<system_t> rrts;

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
