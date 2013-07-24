#include "mvsystem.h"

int main()
{
  typedef mvsystem_c<dubins_c, map_c<3>, cost_c<4>, automaton_product_c<2> > system_t;
  typedef system_t::state state;
  typedef typename system_t::control control;
  typedef typename system_t::trajectory trajectory;
  typedef typename system_t::region_t region;
  
  automaton_product_c<2> abar;
  automaton_ss_c psi(false, SIDEWALK, 1, 0);
  abar.insert(psi, 2);

  label_c l2(SIDEWALK);
  timed_word_c tw(1, l2);
  tw.print();
  auto c = abar.get_cost(tw);
  c.print(cout, "(", ")\n");

  return 0;
}
